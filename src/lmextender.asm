ORG 0x100

SECTION .text

%macro SET_INTERRUPT_ENTRY 1
    push interrupt_entry_%{1}
    push %{1}
    call set_idt_entry
    add sp, 4
%endmacro

dos_entry:
    BITS 16

    xchg bx, bx

    mov sp, stack_top
    
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
    je .continue
    
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
        mov dx, stack_top - dos_entry + 16
        shr dx, 4
        int 0x21

set_idt_entry:
    BITS 16

    push bp
    mov bp, sp

    mov eax, idt
    xor ecx, ecx
    mov cx, [bp + 4]
    shl ecx, 4
    add eax, ecx

    mov ecx, [rm_base]
    xor edx, edx
    mov dx, [bp + 6]
    add ecx, edx

    mov [eax], cx
    shr cx, 16
    mov [eax + 6], cx

    mov word [eax + 2], 0x28
    mov word [eax + 4], 0b1000111000000001

    pop bp
    
    ret

rm_services_entry:
    BITS 16
    
    xchg bx, bx
    
    mov cs:[rm_stack], sp
    mov cs:[rm_stack + 2], ss

    mov sp, stack_top
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

    mov ah, 0x05
    call far [xms_call_address]

    sgdt [original_gdtr]
    sidt [original_idtr]
    
    lgdt [lmextender_gdtr]
    
    call enter_pm32
    BITS 32
    
    call get_eip
    mov eax, [eax + rm_base - $]
        
    pushf
    and dword [esp], ~(1 << 14)
    popf
    
    xor ecx, ecx
    mov cx, [eax + rm_stack]
    xor edx, edx
    mov dx, [eax + rm_stack + 2]
    
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

    lgdt [original_gdtr]

    mov ah, 0x06
    call far [xms_call_address]

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

        mov ax, 0x30
        ltr ax
        lidt [rel lmextender_idtr]

        mov eax, [rsp]
        add rsp, 4

        xor rcx, rcx
        mov cx, [rax]
        xor rdx, rdx
        mov dx, [rax + 2]
        
        shl rdx, 4
        add rdx, rcx
        
        sub rax, 6

        push 0x0
        push rax
        pushf
        push 0x28
        push rdx
        
        iretq

rm_services_handler:
    BITS 32
    
    mov eax, [esp + 4]
    
    cmp word [eax + 6], 0
    je .setup

    cmp word [eax + 6], 1
    je .enter
    
    .setup:
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
        
        mov eax, 0
        ret
    
    .enter:
        mov eax, 1
        ret

%macro INTERRUPT_ENTRY 1
    interrupt_entry_%{1}:
        BITS 64
        
        push qword 0
        push qword %{1}
        jmp interrupt_handler
%endmacro

%macro INTERRUPT_ENTRY_ERROR_CODE 1
    interrupt_entry_%{1}:
        BITS 64

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
INTERRUPT_ENTRY_ERROR_CODE 8
INTERRUPT_ENTRY 9
INTERRUPT_ENTRY_ERROR_CODE 10
INTERRUPT_ENTRY_ERROR_CODE 11
INTERRUPT_ENTRY_ERROR_CODE 12
INTERRUPT_ENTRY_ERROR_CODE 13
INTERRUPT_ENTRY_ERROR_CODE 14
INTERRUPT_ENTRY 16
INTERRUPT_ENTRY_ERROR_CODE 17
INTERRUPT_ENTRY 18
INTERRUPT_ENTRY 19
INTERRUPT_ENTRY 20
INTERRUPT_ENTRY_ERROR_CODE 21

INTERRUPT_ENTRY 129

interrupt_handler:
    BITS 64

    cmp dword [rsp], 32
    jl .exception_handler
    
    cmp dword [rsp], 48
    jl .irq_handler

    cmp dword [rsp], 129
    je .lm_services

    .exception_handler:
        hlt
        
    .irq_handler:
        call rm_interrupt

        jmp .exit

    .lm_services:
        cmp dword [rsp + 40], 0

        je .exit

    .exit:
        add rsp, 16
        iret

rm_interrupt:
    BITS 64

    call return_pm32
    BITS 32
    
    call return_rm
    BITS 16

    lgdt [original_gdtr]
    lidt [original_idtr]

    mov eax, [esp + 8]
    mov byte [interrupt + 1], al

    interrupt:
        db 0xCD
        db 0

    lgdt [lmextender_gdtr]

    call enter_pm32
    BITS 32
    
    call enter_lm
    BITS 64
    
    mov ax, 0x30
    ltr ax
    lidt [rel lmextender_idtr]

    ret
    
enter_pm32:
    BITS 16
    
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
        
        mov cx, 0x10

        mov ds, cx
        mov es, cx
        mov fs, cx
        mov gs, cx
        mov ss, cx

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
            
            jmp ecx
  
enter_lm:
    BITS 32

    call get_eip
    mov eax, [eax + rm_base - $]

    mov [eax + jump_address], eax
    add dword [eax + jump_address], .lm
    mov word [eax + jump_address + 4], 0x28

    mov ecx, cr0
    or ecx, 1 << 31
    mov cr0, ecx

    jmp far [eax + jump_address]

    .lm:
        BITS 64
        
        mov esp, esp
        mov ecx, [rsp]
        add rsp, 4

        jmp rcx
        
return_pm32:
    BITS 64

    mov qword [rel jump_address], .cm32
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
        mov eax, cr0

        ret 4

get_eip:
    BITS 32

    mov eax, [esp]
    ret

SECTION .data

welcome_message db "LMExtender x86_64 DOS Extender", "$"
no_xms_message db "XMS services are not available!", "$"

jump_address times 10 db 0
xms_call_address dd 0

rm_base dd 0
rm_stack dd 0

original_gdtr times 6 db 0
original_idtr times 6 db 0

ALIGN 8

gdt:
    dq 0 ;Null Descriptor

    ;16-Bit Code Descriptor
    dw 0xFFFF
    pm16_base_code times 3 db 0
    db 0b10011011
    db 0b00001111
    db 0

    ;16-Bit Data Descriptor
    dw 0xFFFF
    pm16_base_data times 3 db 0
    db 0b10010011
    db 0b00001111
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
