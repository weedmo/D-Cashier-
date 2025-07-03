# constants.py

import os
from ament_index_python.packages import get_package_share_directory

# 패키지 이름은 실제 사용하시는 이름으로 변경하세요
OBB_PKG     = get_package_share_directory('OBB')
GUI_PKG     = get_package_share_directory('gui')
JSON_PATH   = os.path.join(GUI_PKG, 'resource', 'product_data.json')

# topic_name 
GUI_TOPIC       = '/gui_command'
KEYWORD_TOPIC   = '/keyword'
CLASS_TOPIC     = '/class_name'

# client_name
KEYWORD_SERVICE = '/get_keyword'
OBJ_SERVICE     = '/obj_detect'
ADULT_SERVICE   = '/adult_event'
CHECK_SERVICE   = '/check'
CO_SERVICE      = '/cancel_object'
PAP_ACTION      = '/pick_and_place'

# Robot information
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY =  100
ACC = 100
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT =  "502"

# offset
DEPTH_OFFSET  = -20.0
BOTTLE_OFFSET = -40.0
MIN_DEPTH     = 2.0
YAW_OFFSET    = -50
GRIP_OFFSET   = 200
BUCKET_POS    = [647.10, 74.75, 200, 174.23, 179.03, -157.62]
CANCEL_POS    = [647.10, -130.18, 200, 174.23, -179.03, -157.62]
HOME          = [-9.90, 18.0, 27.19, -0.07, 133.84, -8.53]
PROMPTS       = ['정지', '계산', '완료', '네', '아니오']
KOR2ENG_DICT = {
    "박카스":  "bacchus",
    "참크레커": "cham_cracker",
    "에너지바": "energy_bar",
    "자유시간": "free_time",
    "오레오":   "Oereo",
    "테라":    "terra",
    "영양갱":   "Yeonyanggang"
}
