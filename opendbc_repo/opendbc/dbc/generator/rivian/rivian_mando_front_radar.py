#!/usr/bin/env python3


def generate():
  parts = []
  parts.append("""
VERSION ""


NS_ :
    NS_DESC_
    CM_
    BA_DEF_
    BA_
    VAL_
    CAT_DEF_
    CAT_
    FILTER
    BA_DEF_DEF_
    EV_DATA_
    ENVVAR_DATA_
    SGTYPE_
    SGTYPE_VAL_
    BA_DEF_SGTYPE_
    BA_SGTYPE_
    SIG_TYPE_REF_
    VAL_TABLE_
    SIG_GROUP_
    SIG_VALTYPE_
    SIGTYPE_VALTYPE_
    BO_TX_BU_
    BA_DEF_REL_
    BA_REL_
    BA_DEF_DEF_REL_
    BU_SG_REL_
    BU_EV_REL_
    BU_BO_REL_
    SG_MUL_VAL_

BS_:

BU_: XXX
    """)

  for a in range(0x500, 0x500 + 32):
    parts.append(f"""
BO_ {a} RADAR_TRACK_{a:x}: 8 RADAR
 SG_ CHECKSUM : 0|8@1+ (1,0) [0|255] "" XXX
 SG_ COUNTER : 11|4@0+ (1,0) [0|15] "" XXX
 SG_ UNKNOWN_1 : 23|8@0- (1,0) [-128|127] "" XXX
 SG_ AZIMUTH : 28|10@0- (0.1,0) [-51.2|51.1] "" XXX
 SG_ STATE : 31|3@0+ (1,0) [0|7] "" XXX
 SG_ LONG_DIST : 34|11@0+ (0.1,0) [0|204.7] "" XXX
 SG_ MODE : 55|2@0+ (1,0) [0|3] "" XXX
 SG_ REL_SPEED : 53|14@0- (0.01,0) [-81.92|81.92] "m/s" XXX
    """)

  for a in range(0x500, 0x500 + 32):
    parts.append(f"""
VAL_ {a} STATE 0 "Empty" 1 "New" 2 "New_updated" 3 "Updated" 4 "Coasting" 7 "New_coasting" ;
VAL_ {a} MODE 0 "None" 1 "SRR" 2 "LRR" 3 "SRR_and_LRR" ;
    """)

  parts.append("""
BO_ 532 CAMERA_LEAD_STATUS_A: 8 XXX
 SG_ COUNTER : 1|2@0+ (1,0) [0|3] "" XXX
 SG_ VIDEO_OBJECT_COUNT : 7|5@0+ (1,0) [0|31] "" XXX
 SG_ DAY_TIME_INDICATOR : 9|2@0+ (1,0) [0|3] "" XXX
 SG_ DRIVING_SIDE : 11|1@0+ (1,0) [0|1] "" XXX
 SG_ RIGHT_CLOSE_CUTIN : 12|1@0+ (1,0) [0|1] "" XXX
 SG_ LEFT_CLOSE_CUTIN : 13|1@0+ (1,0) [0|1] "" XXX
 SG_ LEAD_LOST_STATE : 15|2@0+ (1,0) [0|2] "" XXX
 SG_ ID_ALT : 16|7@0+ (1,0) [0|127] "" XXX
 SG_ ID_LEAD : 23|7@0+ (1,0) [0|127] "" XXX
 SG_ LEAD_LONG_DIST : 25|10@0+ (0.01,0) [0|10.23] "m" XXX
 SG_ NEIGHBOR_ID_LEFT : 47|7@0+ (1,0) [0|127] "" XXX
 SG_ NEIGHBOR_ID_RIGHT : 55|7@0+ (1,0) [0|127] "" XXX
 SG_ SAFETY_SUPPRESS : 59|4@0+ (1,0) [0|10] "" XXX
 SG_ VIDEO_VEHICLE_COUNT : 63|4@0+ (1,0) [0|10] "" XXX

BO_ 533 CAMERA_LEAD_STATUS_B: 8 XXX
 SG_ FLAG_0 : 0|1@0+ (1,0) [0|1] "" XXX
 SG_ FLAG_1 : 1|1@0+ (1,0) [0|1] "" XXX
 SG_ FLAG_2 : 2|1@0+ (1,0) [0|1] "" XXX
 SG_ FLAG_3 : 3|1@0+ (1,0) [0|1] "" XXX
 SG_ FLAG_4 : 4|1@0+ (1,0) [0|1] "" XXX
 SG_ FLAG_5 : 5|1@0+ (1,0) [0|1] "" XXX
 SG_ FLAG_6 : 6|1@0+ (1,0) [0|1] "" XXX
 SG_ LEAD_PRESENT : 7|1@0+ (1,0) [0|1] "" XXX
 SG_ FLAG_8 : 8|1@0+ (1,0) [0|1] "" XXX
 SG_ FLAG_9 : 9|1@0+ (1,0) [0|1] "" XXX
 SG_ FLAG_10 : 10|1@0+ (1,0) [0|1] "" XXX
 SG_ FLAG_12 : 12|1@0+ (1,0) [0|1] "" XXX
 SG_ ALERT_MODE : 15|2@0+ (1,0) [0|3] "" XXX
 SG_ SUPPRESS_1 : 19|4@0+ (1,0) [0|15] "" XXX
 SG_ SUPPRESS_2 : 23|4@0+ (1,0) [0|15] "" XXX
 SG_ SUPPRESS_REASONS : 27|20@0+ (1,0) [0|1048575] "" XXX
 SG_ COUNTER : 31|2@0+ (1,0) [0|3] "" XXX
 SG_ SYNC_INDEX_1 : 55|8@0+ (1,0) [0|255] "" XXX
 SG_ SYNC_INDEX_2 : 63|8@0+ (1,0) [0|255] "" XXX

VAL_ 532 LEAD_LOST_STATE 0 "No_Loss" 1 "Lost_Out_Of_View" 2 "Lost_In_View" ;
VAL_ 533 LEAD_PRESENT 0 "No_Lead" 1 "Lead_In_Path" ;
    """)

  return {"rivian_mando_front_radar.dbc": "".join(parts)}
