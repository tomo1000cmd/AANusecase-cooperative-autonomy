# Status of a single task
uint32 timestamp # the time when the task was created
int16 uuid # unique identifier of the task 
string task_status
string task_api # api task name
uint8[] task_info # encoded task info this can be a path or a composed task or a path with start and stop event



#Constant for task status
string TASK_STATUS_TODO = 'todo'
string TASK_STATUS_TODO_TAKEOVER = 'todo#'
string TASK_STATUS_STARTED = 'started'
string TASK_STATUS_SUCCESS = 'finished succesfully'
string TASK_STATUS_FAILURE = 'finished with error'
# Constant for task name
string TASK_API_PATH_FOLLOWING = 'path_following'
string TASK_API_COMPOSED_TASK = 'composed_task'
string TASK_API_STOP_CURRENT_TASK = 'STOP'
string TASK_API_MOTION_CONTROL = 'motion_control'