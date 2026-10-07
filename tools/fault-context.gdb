# Read-only inspection of the paused pre-fix Debug image.
# Source from the CLion GDB console; never reset or continue here.
p/x g_fault_snapshot
p *(TCB_t*)fusion_task_handle
x/16wx ((TCB_t*)fusion_task_handle)->pxStack
p/x fusion_context
x/16wx pxCurrentTCB->pxStack
