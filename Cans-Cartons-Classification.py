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
    lib_path = pathlib.Path(__file__).resolve().parent / "Library_Robot" / "lib3360.py" 
    spec = importlib.util.spec_from_file_location("lib3360", str(lib_path)) 
    lib = importlib.util.module_from_spec(spec) 
    spec.loader.exec_module(lib) 
    motor = getattr(lib, "motor") 
    servo = getattr(lib, "servo") 

PORT = os.getenv("SENDSERIAL_PORT") if os.getenv("SENDSERIAL_PORT") else (sys.argv[1] if len(sys.argv) > 1 else None) 

# =================================== 
# AI Model & Constants 
# =================================== 
os.environ.setdefault("DISPLAY", ":0") 
m_Model_01 = YOLO("./IC_AI_Project_new/IC_AI_Model_06/result/train/weights/best.pt")  # Load custom YOLO model

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

# Servo positions (gripper open/close)
SERVO_OPEN_1 = 1000 
SERVO_OPEN_2 = 1400 
SERVO_CLOSE_1 = 1400 
SERVO_CLOSE_2 = 1400 

# Width thresholds for near/far decisions
DROP_NEAR_WIDTH = 166   # When ball is close enough to drop item
PICK_NEAR_WIDTH = 500   # When item is close enough to grab

CAM_CX = 400            # Camera frame center X (800px width)
CAM_TOL = 70            # Tolerance for considering object centered

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

def get_leftmost_target(detections): # Return the detection with leftmost target
    if not detections: return None 
    return min(detections, key=lambda d: d["cx"]) 

def label_norm(s): # Normalize label string for comparison
    return (s or "").strip().lower() 

def is_aluminum(label): # Check if the label corresponds to an aluminum can
    l = label_norm(label) 
    return ("mini-cola-cans" in l) or ("sprite" in l) or ("aluminum" in l) 

def is_carton(label): # Check if the label corresponds to a carton
    l = label_norm(label) 
    return ("vitasoy-soyabean-milk" in l) or ("lemon-tea" in l) or ("carton" in l) 

def zone_from_flag(label): # Return the zone type ('aluminum' or 'carton') based on ball color
    l = label_norm(label) 
    if "green_ball" in l: return "aluminum" 
    if "red_ball" in l: return "carton" 
    return None 

# =================================== 
# Initialization 
# =================================== 
servo(SERVO_OPEN_1, SERVO_OPEN_2, mode="hex")  # Open gripper initially
stop_all() 

# State machine variables
state = "INIT_FORWARD" # States: INIT_FORWARD -> SEARCH_ITEM -> POST_GRAB_BACKWARD -> POST_GRAB_TURN -> MOVE_TO_ZONE -> POST_DROP_BACKWARD -> POST_DROP_TURN_(R/L) -> MISSION_DONE
state_timer = None # Timer for state duration
counts = {"aluminum": 0, "carton": 0} # Counters for collected items
current_target_type = None # Currently targeted item type ("aluminum" or "carton")
scan_timer = None # Timer for scanning sequence
scan_phase = None # Scanning sub-phase: None, "TURN_L", "HOLD_L", "TURN_R", "HOLD_R"

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
            distance_label = "Near" if obj_width > 450 else "Far"     # Threshold can be adjusted
            (dw, dh), _ = cv2.getTextSize(distance_label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)  
            cv2.putText(annotated_frame, distance_label,  
                        ((frame_w - dw)//2, frame_h - 20),  
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)  
            break   # only draw one distance label per frame (the first detection)  

    # Collect detection data for state machine
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
            state = "SEARCH_ITEM" 
            state_timer = time.time() 

    elif state == "SEARCH_ITEM": # Decide which category to collect next: aluminum first, then cartons
        target_category = "aluminum" if counts["aluminum"] < 2 else "carton" 
        candidates = [] 
        for d in dets: 
            if target_category == "aluminum" and is_aluminum(d["class_name"]): candidates.append(d) 
            elif target_category == "carton" and is_carton(d["class_name"]): candidates.append(d) 

        best = get_leftmost_target(candidates) 
        if best: 
            search_timer = None 
            target_x = best["cx"] 
            # Center on the object
            if target_x < CAM_CX - CAM_TOL: curve_left() 
            elif target_x > CAM_CX + CAM_TOL: curve_right() 
            else:  
                if best["width"] < PICK_NEAR_WIDTH:   # State is Far -> move forward
                    forward() 
                else:                                 # State is Near -> grab
                    stop_all() 
                    servo(SERVO_CLOSE_1, SERVO_CLOSE_2, mode="hex") 
                    current_target_type = target_category 
                    state = "POST_GRAB_BACKWARD" 
                    state_timer = time.time() 
        else: 
            stop_all() 

    elif state == "POST_GRAB_BACKWARD": # Reverse after grabbing an item
        if elapsed < 7.0: 
            backward() 
        else: 
            state = "POST_GRAB_TURN" 
            state_timer = time.time() 

    elif state == "POST_GRAB_TURN": # Turn toward the corresponding drop zone
        if elapsed < 1.6: 
            if current_target_type == "aluminum": turn_left_spot() 
            else: turn_right_spot() 
        elif elapsed < 1.8: 
            forward() 
        else: 
            stop_all() 
            state = "MOVE_TO_ZONE" 
            state_timer = time.time() 

    elif state == "MOVE_TO_ZONE": # Approach the ball (flag) of the correct color and drop the item
        accept_dets = scan_phase in (None, "HOLD_L", "HOLD_R")
        candidates = [d for d in dets if accept_dets and (zone_from_flag(d["class_name"]) == current_target_type)] 
        best = get_leftmost_target(candidates) 

        if best: 
            scan_timer = None
            scan_phase = None
            cx = best["cx"] 
            n = counts[current_target_type]
            # Desired X position relative to ball depends on item type and count
            if current_target_type == "aluminum":
                desired_x = 655 if n == 0 else 150
            else:
                desired_x = 150 if n == 0 else 655

            if cx < desired_x - CAM_TOL: curve_left()
            elif cx > desired_x + CAM_TOL: curve_right()
            else: 
                if best["width"] < DROP_NEAR_WIDTH: 
                    forward() 
                else: 
                    stop_all() 
                    servo(SERVO_OPEN_1, SERVO_OPEN_2, mode="hex")   # Drop item
                    counts[current_target_type] += 1 
                    # Check if mission complete
                    if counts["aluminum"] >= 2 and counts["carton"] >= 2: 
                        state = "POST_DROP_BACKWARD" 
                        state_timer = time.time() 
                    else:
                        state = "POST_DROP_BACKWARD" 
                        state_timer = time.time() 
        else: # Scanning routine: turn left 1s, hold, turn right 2s, hold -> repeat until find the ball
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
                if scan_elapsed < 1.0:
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
                if scan_elapsed < 1.0:
                    stop_all()
                else:
                    scan_timer = time.time()
                    scan_phase = "TURN_L"

    elif state == "POST_DROP_BACKWARD": # Reverse after dropping, decide next turn or finish
        if elapsed < 5.0:
            backward()
        else:
            stop_all()
            if counts["aluminum"] >= 2 and counts["carton"] >= 2:
                state = "MISSION_DONE"
                state_timer = time.time()
            elif current_target_type == "aluminum":
                state = "POST_DROP_TURN_RIGHT"
                state_timer = time.time()
            else:
                state = "POST_DROP_TURN_LEFT"
                state_timer = time.time()

    elif state == "POST_DROP_TURN_RIGHT":
        if elapsed < 1.6:
            turn_right_spot()
        else:
            stop_all()
            state = "SEARCH_ITEM"
            state_timer = time.time()

    elif state == "POST_DROP_TURN_LEFT":
        if elapsed < 1.8:
            turn_left_spot()
        else:
            stop_all()
            state = "SEARCH_ITEM"
            state_timer = time.time()

    elif state == "MISSION_DONE": # Final sequence: back up, turn, then stop completely
        if elapsed < 3.0:
            backward()
        elif elapsed < 4.8:
            turn_left_spot()
        else:
            stop_all()
            servo(SERVO_OPEN_1, SERVO_OPEN_2, mode="hex")
            motor(0, 0, dir1=0, dir2=0, mode="once", port=PORT)
            break

    # Display state and counters on frame
    cv2.putText(annotated_frame, f"STATE: {state} | AL: {counts['aluminum']} | CA: {counts['carton']}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2) 
    if state == "MISSION_DONE": 
        cv2.putText(annotated_frame, "FINISHED!", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 

    cv2.imshow("Game 2", annotated_frame) 

    # Ensure stop and exit if mission done (redundant safety)
    if state == "MISSION_DONE": 
        servo(SERVO_OPEN_1, SERVO_OPEN_2, mode="hex") 
        motor(0, 0, dir1=0, dir2=0, mode="once", port=PORT) 
        break 

    # Quit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord("q"): 
        servo(SERVO_OPEN_1, SERVO_OPEN_2, mode="hex") 
        motor(0, 0, dir1=0, dir2=0, mode="once", port=PORT)         
        break 

cap.release() 
cv2.destroyAllWindows()