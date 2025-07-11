%macro STACK_FRAME_CREATE_16 0
push bp
mov bp, sp
%endmacro

%macro STACK_FRAME_DESTROY_16 0
mov sp, bp
pop bp
%endmacro

%macro STACK_FRAME_CREATE_32 0
push ebp
mov ebp, esp
%endmacro

%macro STACK_FRAME_DESTROY_32 0
mov esp, ebp
pop ebp
%endmacro

%macro STACK_FRAME_CREATE_64 0
push rbp
mov rbp, rsp
%endmacro

%macro STACK_FRAME_DESTROY_64 0
mov rsp, rbp
pop rbp
%endmacro

