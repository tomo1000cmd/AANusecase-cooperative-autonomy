string vehicle_id
int16 operation # look the constant for operation type

# The task it gives to the vehicle operating agent
Task task
Task[] tasks

##Constant define
##time table related operation
int16 OPERATION_ADD_TIMETABLE = 1
int16 OPERATION_REMOVE_TIMETABLE = 2
int16 OPERATION_ALTER_TIMETABLE = 3
int16 OPERATION_RESUME_TIMETABLE = 4
int16 OPERATION_CLEAR_TIMETABLE = 5
int16 OPERATION_EXECUTE_NOW = 6
int16 OPERATION_REPLACE_ENTIRE_TIMETABLE = 7
int16 OPERATION_ASK_FOR_URGENT_ID = 99




