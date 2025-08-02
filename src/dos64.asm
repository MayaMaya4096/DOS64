ORG 0x100

SECTION .text

%macro SET_INTERRUPT_ENTRY 1
    push dword interrupt_entry_%{1}
    push dword %{1}
    call set_idt_entry
    add sp, 4
%endmacro

dos_entry:
    BITS 16

    xchg bx, bx

    mov esp, stack_top
    
    mov ah, 0x09
    mov dx, welcome_message
    int 0x21
    
    mov eax, cs
    shl eax, 4

    mov [rm_base], eax

    mov ecx, eax
    mov [pm16_base_code], cx
    mov [pm16_base_data], cx

    shr ecx, 16
    mov [pm16_base_code + 2], cl
    mov [pm16_base_data + 2], cl

    mov ecx, eax
    add ecx, tss
    
    mov [tss_base], cx
    shr ecx, 16
    mov [tss_base + 2], cl
    
    add [lmextender_gdtr + 2], eax
    add [ist1], eax
    add [lmextender_idtr + 2], eax

    mov ax, 0x4300
    int 0x2F

    cmp al, 0x80
    jmp .continue
    
    mov ah, 0x09
    mov dx, no_xms_message
    int 0x21

    mov ah, 0x4C
    mov al, 1
    int 0x21

    .continue:
        mov ax, 0x4310
        int 0x2F

        mov [xms_call_address], bx
        mov [xms_call_address + 2], es

        SET_INTERRUPT_ENTRY 0
        SET_INTERRUPT_ENTRY 1
        SET_INTERRUPT_ENTRY 2
        SET_INTERRUPT_ENTRY 3
        SET_INTERRUPT_ENTRY 4
        SET_INTERRUPT_ENTRY 5
        SET_INTERRUPT_ENTRY 6
        SET_INTERRUPT_ENTRY 7
        SET_INTERRUPT_ENTRY 8
        SET_INTERRUPT_ENTRY 9
        SET_INTERRUPT_ENTRY 10
        SET_INTERRUPT_ENTRY 11
        SET_INTERRUPT_ENTRY 12
        SET_INTERRUPT_ENTRY 13
        SET_INTERRUPT_ENTRY 14
        SET_INTERRUPT_ENTRY 16
        SET_INTERRUPT_ENTRY 17
        SET_INTERRUPT_ENTRY 18
        SET_INTERRUPT_ENTRY 19
        SET_INTERRUPT_ENTRY 20
        SET_INTERRUPT_ENTRY 21

        SET_INTERRUPT_ENTRY 129

        mov ah, 0x25
        mov al, 0x80
        mov dx, rm_services_entry
        int 0x21
    
        mov ah, 0x31
        mov al, 0
        mov dx, stack_top + 16
        shr dx, 4
        int 0x21

set_idt_entry:
    BITS 16

    mov eax, idt
    xor ecx, ecx
    mov cl, [esp + 2]
    shl ecx, 4
    add eax, ecx

    mov ecx, [rm_base]
    xor edx, edx
    mov dx, [esp + 6]
    add ecx, edx

    mov [eax], cx
    shr ecx, 16
    mov [eax + 6], cx

    mov word [eax + 2], 0x28
    mov word [eax + 4], 0b1000111000000001
    
    ret

rm_services_entry:
    BITS 16
    
    xchg bx, bx
    
    mov cs:[rm_stack], sp
    mov cs:[rm_stack + 2], ss

    mov esp, stack_top
    mov ax, cs
    mov ss, ax

    push ds
    push es
    push fs
    push gs

    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    
    sgdt [original_gdtr]
    sidt [original_idtr]
        
    call enter_pm32
    BITS 32
    
    call get_eip
    lea eax, [eax + rm_stack - $]
    
    xor ecx, ecx
    mov cx, [eax]
    xor edx, edx
    mov dx, [eax + 2]
    
    shl edx, 4
    add edx, ecx
    
    push edx
    call rm_services_handler

    xchg bx, bx

    cmp eax, 0
    jne .iret_lm

    add esp, 4
    
    call return_rm
    BITS 16

    pop gs
    pop fs
    pop es
    pop ds
    
    mov sp, cs:[rm_stack]
    mov ss, cs:[rm_stack + 2]
    
    iret
    
    .iret_lm:
        BITS 32
        
        call enter_lm
        BITS 64

        call load_tr
        lidt [rel lmextender_idtr]

        mov eax, [rsp]
        add rsp, 4

        xor rcx, rcx
        mov cx, [rax]
        xor rdx, rdx
        mov dx, [rax + 2]
        
        shl rdx, 4
        add rdx, rcx
        
        mov cx, [rax + 4]
        add rax, 6

        push 0x0
        push rax
        pushf
        mov [rsp], cx
        push 0x28
        push rdx

        iretq

rm_services_handler:
    BITS 32
    
    mov eax, [esp + 4]
    
    cmp word [eax + 6], 0
    je .enter

    .enter:
        push edi

        mov edi, 0x200000
        xor eax, eax
        mov ecx, 3 * 4096
        rep stosb

        mov edi, 0x200000
        mov dword [edi], 0x201000 | 0b11

        mov edi, 0x201000
        mov dword [edi], 0x202000 | 0b11

        mov edi, 0x202000
        mov eax, 0x203000
        mov ecx, 8

        .fill_pml2:
            mov edx, eax
            or edx, 0b11
            mov dword [edi], edx

            add eax, 0x1000
            add edi, 8

            loop .fill_pml2

        mov edi, 0x203000
        mov eax, 0
        mov ecx, 512 * 8

        .fill_pml1:
            mov edx, eax
            or edx, 0b11
            mov dword [edi], edx

            add eax, 0x1000
            add edi, 8

            loop .fill_pml1

        pop edi

        mov eax, 0x200000
        mov cr3, eax

        mov eax, cr4
        or eax, 1 << 5
        mov cr4, eax
    
        mov ecx, 0xC0000080
        rdmsr
        or eax, 1 << 8
        wrmsr
        
        mov eax, 1
        ret
        
%macro INTERRUPT_ENTRY 1
    interrupt_entry_%{1}:
        BITS 64

        cmp qword [rsp + 16], 0x28
        je .error_code

        push qword -1
        
        .error_code:
            push qword %{1}
            jmp interrupt_handler
%endmacro

INTERRUPT_ENTRY 0
INTERRUPT_ENTRY 1
INTERRUPT_ENTRY 2
INTERRUPT_ENTRY 3
INTERRUPT_ENTRY 4
INTERRUPT_ENTRY 5
INTERRUPT_ENTRY 6
INTERRUPT_ENTRY 7
INTERRUPT_ENTRY 8
INTERRUPT_ENTRY 9
INTERRUPT_ENTRY 10
INTERRUPT_ENTRY 11
INTERRUPT_ENTRY 12
INTERRUPT_ENTRY 13
INTERRUPT_ENTRY 14
INTERRUPT_ENTRY 16
INTERRUPT_ENTRY 17
INTERRUPT_ENTRY 18
INTERRUPT_ENTRY 19
INTERRUPT_ENTRY 20
INTERRUPT_ENTRY 21

INTERRUPT_ENTRY 129

interrupt_handler:
    BITS 64
    
    push gs
    push fs

    sub rsp, 8
    mov [rsp], es

    sub rsp, 8
    mov [rsp], ds

    push r15
    push r14
    push r13
    push r12
    push r11
    push r10
    push r9
    push r8
    push rbp
    push rdi
    push rsi
    push rdx
    push rcx
    push rbx
    push rax
    
    cmp qword [rsp + 160], -1
    jne .exception_handler
    
    cmp byte [rsp + 152], 8
    jb .exception_handler
    
    cmp byte [rsp + 152], 16
    jb .irq_handler

    cmp byte [rsp + 152], 112
    jb .exception_handler
    
    cmp byte [rsp + 152], 120
    jb .irq_handler
    
    cmp byte [rsp + 152], 129
    je .lm_services

    .exception_handler:
        hlt
        
    .irq_handler:
        mov dil, [rsp + 152]
        call rm_irq

        jmp .exit

    .lm_services:        
        cmp rdi, 0
        je .interrupt

        .interrupt:
            mov al, sil

            mov rsi, rdx
            lea rdi, [rel rm_registers]
            mov rcx, 38
            rep movsb

            mov dil, al
            push rdx
            call rm_interrupt
            pop rdx
            
            lea rsi, [rel rm_registers]
            mov rdi, rdx
            mov rcx, 38
            rep movsb

            jmp .exit
            
    .exit:
        pop rax
        pop rbx
        pop rcx
        pop rdx
        pop rsi
        pop rdi
        pop rbp
        pop r8
        pop r9
        pop r10
        pop r11
        pop r12
        pop r13
        pop r14
        pop r15
        
        mov ds, [rsp]
        add rsp, 8

        mov es, [rsp]
        add rsp, 8

        pop fs
        pop gs

        add rsp, 16
        iretq

rm_interrupt:
    BITS 64

    xchg bx, bx

    push r15
    push r14
    push r13
    push r12
    push rbp
    push rbx

    mov [rel .interrupt + 1], dil
    
    call return_pm32
    BITS 32
    
    call return_rm
    BITS 16
    
    mov eax, [rm_registers]
    mov ebx, [rm_registers + 4]
    mov ecx, [rm_registers + 8]
    mov edx, [rm_registers + 12]
    mov esi, [rm_registers + 16]
    mov edi, [rm_registers + 20]
    mov ebp, [rm_registers + 24]

    mov ds, [rm_registers + 28]
    mov es, [rm_registers + 30]
    mov fs, [rm_registers + 32]
    mov gs, [rm_registers + 34]

    .interrupt:
        db 0xCD
        db 0

    xchg bx, bx
    
    mov [rm_registers], eax
    mov [rm_registers + 4], ebx
    mov [rm_registers + 8], ecx
    mov [rm_registers + 12], edx
    mov [rm_registers + 16], esi
    mov [rm_registers + 20], edi
    mov [rm_registers + 24], ebp

    mov [rm_registers + 28], ds
    mov [rm_registers + 30], es
    mov [rm_registers + 32], fs
    mov [rm_registers + 34], gs
    
    pushf
    pop ax
    mov [rm_registers + 36], ax

    mov ax, cs
    
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax

    call enter_pm32
    BITS 32
        
    call enter_lm
    BITS 64
        
    call load_tr
    lidt [rel lmextender_idtr]

    pop rbx
    pop rbp
    pop r12
    pop r13
    pop r14
    pop r15

    ret

rm_irq:
    BITS 64

    push r15
    push r14
    push r13
    push r12
    push rbp
    push rbx

    mov [rel .irq + 1], dil

    call return_pm32
    BITS 32

    call return_rm
    BITS 16
    
    .irq:
        db 0xCD
        db 0

    call enter_pm32
    BITS 32

    call enter_lm
    BITS 64

    call load_tr
    lidt [rel lmextender_idtr]

    pop rbx
    pop rbp
    pop r12
    pop r13
    pop r14
    pop r15
    
    ret
    
enter_pm32:
    BITS 16
    
    mov ax, 0x2401
    int 0x15
    
    lgdt [lmextender_gdtr]
    
    mov eax, [rm_base]
    
    mov [jump_address], eax
    add dword [jump_address], .pm32
    mov word [jump_address + 4], 0x18

    mov ecx, cr0
    or ecx, 1 << 0
    mov cr0, ecx

    jmp far dword [jump_address]

    .pm32:
        BITS 32
        
        mov cx, 0x20

        mov ds, cx
        mov es, cx
        mov ss, cx
        mov fs, cx
        mov gs, cx

        and esp, 0xFFFF
        add esp, eax

        xor ecx, ecx
        pop cx
        add ecx, eax

        jmp ecx

return_rm:
    BITS 32
    jmp 0x08:.pm16

    .pm16:
        BITS 16
        
        mov ax, 0x10

        mov ds, ax
        mov es, ax
        mov fs, ax
        mov gs, ax
        mov ss, ax

        mov eax, [rm_base]
        sub esp, eax
        
        pop ecx
        sub ecx, eax
        
        mov word [jump_address], .rm
        shr eax, 4
        mov [jump_address + 2], ax
        
        mov edx, cr0
        and edx, ~(1 << 0)
        mov cr0, edx
        
        jmp far [jump_address]
        
        .rm:        
            mov ds, ax
            mov es, ax
            mov fs, ax
            mov gs, ax
            mov ss, ax

            lgdt [original_gdtr]
            lidt [original_idtr]

            mov ax, 0x2400
            int 0x15
            
            jmp ecx
  
enter_lm:
    BITS 32

    mov ecx, cr0
    or ecx, 1 << 31
    mov cr0, ecx

    call get_eip
    mov eax, [eax + rm_base - $]
    
    mov [eax + jump_address], eax
    add dword [eax + jump_address], .lm
    mov word [eax + jump_address + 4], 0x28
    
    jmp far [eax + jump_address]

    .lm:
        BITS 64
        
        mov esp, esp
        mov ecx, [rsp]
        add rsp, 4

        jmp rcx
        
return_pm32:
    BITS 64

    lea rax, [rel .cm32]
    mov [rel jump_address], rax
    mov word [rel jump_address + 8], 0x18
    
    jmp far [rel jump_address]

    .cm32:
        BITS 32
        
        mov ax, 0x20

        mov ds, ax
        mov es, ax
        mov fs, ax
        mov gs, ax
        mov ss, ax
        
        mov eax, cr0
        and eax, ~(1 << 31)
        mov cr0, eax

        ret 4

load_tr:
    BITS 64

    mov ax, 0x30
    ltr ax
    and byte [rel gdt + 53], ~(1 << 1)
    
    ret

get_eip:
    BITS 32

    mov eax, [esp]
    ret

SECTION .data

welcome_message db "DOS/64 64-Bit DOS Extender", "$"
no_xms_message db "XMS services are not available!", "$"

jump_address times 10 db 0
xms_call_address dd 0

rm_base dd 0
rm_stack dd 0
rm_registers times 38 db 0

original_gdtr times 6 db 0
original_idtr times 6 db 0

ALIGN 8

gdt:
    dq 0 ;Null Descriptor

    ;16-Bit Code Descriptor
    dw 0xFFFF
    pm16_base_code times 3 db 0
    db 0b10011011
    db 0
    db 0

    ;16-Bit Data Descriptor
    dw 0xFFFF
    pm16_base_data times 3 db 0
    db 0b10010011
    db 0
    db 0

    ;32-Bit Code Descriptor
    dw 0xFFFF
    dw 0
    db 0
    db 0b10011011
    db 0b11001111
    db 0

    ;32-Bit Data Descriptor
    dw 0xFFFF
    dw 0
    db 0
    db 0b10010011
    db 0b11001111
    db 0
    
    ;64-Bit Code Descriptor
    dw 0xFFFF
    dw 0
    db 0
    db 0b10011011
    db 0b10101111
    db 0

    ;64-Bit TSS
    dw idt - tss
    tss_base times 3 db 0
    db 0b10001001
    db 0
    db 0
    dq 0
    
lmextender_gdtr:
    dw lmextender_gdtr - gdt - 1
    dd gdt
    
tss:
    dd 0
    dq 0
    dq 0
    dq 0
    dq 0
    ist1 dd stack_top
    dd 0
    dq 0
    dq 0
    dq 0
    dq 0
    dq 0
    dq 0
    dq 0
    dw 0
    dw 0xFFFF

idt:
    times 256 * 16 db 0

lmextender_idtr:
    dw lmextender_idtr - idt - 1
    dq idt

SECTION .bss

stack_bottom:
resb 4096
stack_top:
