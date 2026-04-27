from ultralytics import YOLO 
from ultralytics.utils.plotting import colors 
import cv2 
import time 
import os 
import sys 

# ===================================       
# Robot Communication Setting 
# =================================== 
try: 
    from Library_Robot.lib3360 import motor, servo 
except Exception: 
    import importlib.util 
    import pathlib 
    lib_path = pathlib.Path(__file__).resolve().parent / 'lib3360.py' 
    spec = importlib.util.spec_from_file_location('lib3360', str(lib_path)) 
    lib = importlib.util.module_from_spec(spec) 
    spec.loader.exec_module(lib) 
    motor = getattr(lib, 'motor') 
    servo = getattr(lib, 'servo') 

PORT = os.getenv("SENDSERIAL_PORT") if os.getenv("SENDSERIAL_PORT") else (sys.argv[1] if len(sys.argv) > 1 else None) 

# =================================== 
# AI Model & Constants 
# =================================== 
os.environ.setdefault("DISPLAY", ":0") 
m_Model_01 = YOLO('./IC_AI_Project_new/IC_AI_Model_06/result/train/weights/best.pt')  # Load custom YOLO model

cap = cv2.VideoCapture(0, cv2.CAP_V4L2) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)   # Set camera width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)  # Set camera height

# Motor PWM values
FWD_PWM_L = 14000      # Left wheel forward speed
FWD_PWM_R = 14000      # Right wheel forward speed
CURVE_FAST = 17000     # Faster wheel for curved movement
CURVE_SLOW = 12000     # Slower wheel for curved movement
TURN_PWM = 12000       # Speed for spot turn
SCAN_TURN_PWM = 10000  # Slower speed for scanning turns

# Basic movement functions
def stop_all(): motor(0, 0, dir1=0, dir2=0, mode="once", port=PORT) 
def forward(): motor(FWD_PWM_L, FWD_PWM_R, dir1=1, dir2=1, mode="once", port=PORT) 
def backward(): motor(15000, 15000, dir1=0, dir2=0, mode="once", port=PORT) 
def curve_left(): motor(CURVE_SLOW, CURVE_FAST, dir1=1, dir2=1, mode="once", port=PORT) 
def curve_right(): motor(CURVE_FAST, CURVE_SLOW, dir1=1, dir2=1, mode="once", port=PORT) 
def turn_left_spot(): motor(TURN_PWM, TURN_PWM, dir1=0, dir2=1, mode="once", port=PORT) 
def turn_right_spot(): motor(TURN_PWM, TURN_PWM, dir1=1, dir2=0, mode="once", port=PORT) 
def scan_turn_left_spot(): motor(SCAN_TURN_PWM, SCAN_TURN_PWM, dir1=0, dir2=1, mode="once", port=PORT) 
def scan_turn_right_spot(): motor(SCAN_TURN_PWM, SCAN_TURN_PWM, dir1=1, dir2=0, mode="once", port=PORT) 

def get_leftmost_target(detections): 
    """Return the detection with the smallest center X coordinate (leftmost)."""
    if not detections: return None 
    return min(detections, key=lambda d: d["cx"]) 

# =================================== 
# Initialization 
# =================================== 
cans_collected = 0                 # Counter for collected cans
task_start_time = time.time()      # Record mission start time (not used further)
servo(1000, 1400, mode='hex')      # Open gripper initially
stop_all() 

# State machine variables
state = "INIT_FORWARD"             # States: INIT_FORWARD -> SEARCH_CAN -> POST_GRAB -> SEARCH_BALL -> POST_DROP 
state_timer = None                 # Timer for state duration
scan_timer = None                  # Timer for scanning sequence
scan_phase = None                  # Scanning sub-phase: None, "TURN_L", "HOLD_L", "TURN_R", "HOLD_R"

while True: 
    ret, frame = cap.read() 
    if not ret: continue 

    if state_timer is None: 
        state_timer = time.time() 

    # Run YOLO detection
    results = m_Model_01(frame, imgsz=(640, 640), conf=0.2) 
    annotated_frame = results[0].plot() 

    # ---------------------------  
    # Draw distance info (width & Far/Near)  
    # ---------------------------  
    frame_h, frame_w = frame.shape[:2]  
    if results[0].boxes is not None:  
        for box, conf, cls_id in zip(results[0].boxes.xyxy.cpu().numpy(),  
                                     results[0].boxes.conf.cpu().numpy(),  
                                     results[0].boxes.cls.cpu().numpy()):  
            x1, y1, x2, y2 = map(int, box)  
            class_name = m_Model_01.names[int(cls_id)].lower()  
            center_x = int((x1 + x2) / 2)  
            center_y = int((y1 + y2) / 2)  
            obj_width = x2 - x1  

            # Draw center point (colored by class)  
            center_color = colors(int(cls_id), bgr=True)  
            cv2.rectangle(annotated_frame, (center_x-5, center_y-5), (center_x+5, center_y+5), center_color, 2)  

            # Draw coordinates below center  
            coords_text = f"({center_x},{center_y})"  
            (tw, th), _ = cv2.getTextSize(coords_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)  
            cv2.putText(annotated_frame, coords_text,  
                        (center_x - tw//2, center_y + 15),  
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, center_color, 1)  

            # Draw object width below coordinates  
            width_text = f"Width: {obj_width}"  
            (ww, wh), _ = cv2.getTextSize(width_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)  
            cv2.putText(annotated_frame, width_text,  
                        (center_x - ww//2, center_y + 30),  
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, center_color, 1)  

            # Determine if object is near or far based on width threshold
            distance_label = "Near" if obj_width > 450 else "Far" # Threshold can be adjusted
            (dw, dh), _ = cv2.getTextSize(distance_label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)  
            cv2.putText(annotated_frame, distance_label,  
                        ((frame_w - dw)//2, frame_h - 20),  
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)  
            break # only draw one distance label per frame (the first detection)  

    # Collect detection data for state machine
    elapsed = time.time() - state_timer 
    dets = [] 
    if results[0].boxes is not None: 
        for box, conf, cls_id in zip(results[0].boxes.xyxy.cpu().numpy(), results[0].boxes.conf.cpu().numpy(), results[0].boxes.cls.cpu().numpy()): 
            x1, y1, x2, y2 = map(int, box) 
            dets.append({ 
                "class_name": m_Model_01.names[int(cls_id)].lower(), 
                "cx": int((x1 + x2) / 2), 
                "width": x2 - x1 
            }) 

    # --------------------------------------------------------- 
    # STATE MACHINE 
    # --------------------------------------------------------- 
    elapsed = time.time() - state_timer 

    if state == "INIT_FORWARD": 
        if elapsed < 3.0: # Move forward for 3 seconds to leave start area
            forward() 
        else: 
            stop_all() 
            state = "SEARCH_CAN" 
            state_timer = time.time() 

    elif state == "SEARCH_CAN": # Look for cola cans (only "mini-cola-cans" class)
        candidates = [d for d in dets if d["class_name"] == "mini-cola-cans"] 
        best = get_leftmost_target(candidates) 
        if best: 
            target_x = best["cx"] 
            # Smooth centering that turn if target is off-center
            if target_x < 300: curve_left() 
            elif target_x > 500: curve_right() 
            else:  
                if best["width"] < 500: # State is Far -> move forward
                    forward() 
                else: # State is Near -> grab
                    stop_all() 
                    servo(1400, 1400, mode='hex') # Close gripper
                    state = "POST_GRAB" 
                    state_timer = time.time() 
        else: 
            stop_all() 

    elif state == "POST_GRAB": # After grabbing a can: reverse, turn to face ball zone, then go forward a bit
        if elapsed < 6.0: 
            backward() 
        elif elapsed < 8.1:
            turn_left_spot() # Rotate toward center area
        elif elapsed < 8.3: 
            forward() 
        else: 
            stop_all() 
            state = "SEARCH_BALL" 
            state_timer = time.time() 

    elif state == "SEARCH_BALL": # Only accept detections during HOLD phases (after each turn completes)
        accept_dets = scan_phase in (None, "HOLD_L", "HOLD_R")
        candidates = [d for d in dets if accept_dets and ("green_ball" in d["class_name"])] 
        best = get_leftmost_target(candidates) 

        if best: 
            scan_timer = None
            scan_phase = None
            cx = best["cx"] 
            # Drop position depends on number of cans already collected:
            # 1st can → drop left of ball (ball on right)
            # 2nd can → drop right of ball (ball on left)
            # 3rd can → drop center (ball centered)
            if cans_collected == 0: 
                desired_x = 643 
            elif cans_collected == 1: 
                desired_x = 145 
            else: 
                desired_x = 390 

            if cx < desired_x - 90: curve_left() 
            elif cx > desired_x + 90: curve_right() 
            else: # Different near threshold for the 2nd can (specific to setup)
                if cans_collected == 1:
                    close_threshold = 158
                else:
                    close_threshold = 157

                if best["width"] < close_threshold:
                    forward()
                else: 
                    stop_all() 
                    servo(1000, 1400, mode='hex') # Open gripper to drop can
                    cans_collected += 1 
                    state = "POST_DROP" 
                    state_timer = time.time() 

        else: # Scanning routine: turn left 1s , hold, turn right 2s, hold -> repeat until find the ball
            if scan_timer is None:
                scan_timer = time.time()
                scan_phase = "TURN_L"

            scan_elapsed = time.time() - scan_timer

            if scan_phase == "TURN_L":
                if scan_elapsed < 1.0:
                    scan_turn_left_spot()
                else:
                    stop_all()
                    scan_timer = time.time()
                    scan_phase = "HOLD_L"
            elif scan_phase == "HOLD_L":
                if scan_elapsed < 0.45:
                    stop_all()
                else:
                    scan_timer = time.time()
                    scan_phase = "TURN_R"
            elif scan_phase == "TURN_R":
                if scan_elapsed < 2.0:
                    scan_turn_right_spot()
                else:
                    stop_all()
                    scan_timer = time.time()
                    scan_phase = "HOLD_R"
            elif scan_phase == "HOLD_R":
                if scan_elapsed < 0.45:
                    stop_all()
                else:
                    scan_timer = time.time()
                    scan_phase = "TURN_L"

    elif state == "POST_DROP": # After dropping a can, reverse, turn right and then either continue or finish
        if elapsed < 5.0: 
            backward() 
        elif elapsed < 6.7: 
            turn_right_spot() 
        else: 
            stop_all() 
            if cans_collected >= 3: # Mission complete after 3 cans
                servo(1000, 1400, mode="hex") 
                motor(0, 0, dir1=0, dir2=0, mode="once", port=PORT) 
                break 
            state = "SEARCH_CAN" # Go back for next can
            state_timer = time.time() 

    # Display state and can count on frame
    cv2.putText(annotated_frame, f"STATE: {state} | CANS: {cans_collected}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2) 
    cv2.imshow("Game 1", annotated_frame) 

    # Quit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        servo(1000, 1400, mode='hex') 
        motor(0, 0, dir1=0, dir2=0, mode='once', port=PORT)         
        break 

cap.release() 
cv2.destroyAllWindows()