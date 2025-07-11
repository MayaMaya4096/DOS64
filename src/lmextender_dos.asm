ORG 0x100

SECTION .text

%include "lmextender_common.inc"

start_dos:
    BITS 16

    ;Load stack
    mov sp, stack_top

    ;Display welcome message
    mov ah, 0x09
    mov dx, welcome_message
    int 0x21

    ;Setup interrupt handler
    mov ah, 0x25
    mov al, 0x80
    mov dx, interrupt_handler
    int 0x21
    
    ;Save base address
    xor ebx, ebx
    mov bx, cs
    shl ebx, 4

    ;Disable interrupts
    cli

    ;Enable A20 Line
    mov ax, 0x2401
    int 0x15
    
    xchg bx, bx

    ;Complete GDT and switch to protected mode
    mov [pm16_code_base], bx
    mov [pm16_data_base], bx

    mov eax, ebx
    shr eax, 16
    mov [pm16_code_base + 2], al
    mov [pm16_data_base + 2], al
    
    add dword [gdtr + 2], ebx
    push ebx

    call dword enter_pm32
    BITS 32
    
    ; Copy upper
    mov esi, ebx
    add esi, start_upper
    mov edi, 0x200000
    mov ecx, end_upper - start_upper
    rep movsb

    call return_rm
    BITS 16
            
    mov ah, 0x31
    mov al, 0
    mov dx, stack_top - start_dos + 16
    shr dx, 4
    int 0x21

enter_pm32:
    BITS 16

    mov eax, [esp + 4]
    add [esp], eax
    
    mov [jump_address], eax
    add dword [jump_address], .pm32
    mov word [jump_address + 4], 0x18
    
    lgdt [gdtr]

    mov ecx, cr0
    or ecx, 1 << 0
    mov cr0, ecx

    jmp far dword [jump_address]
    
    .pm32:
        BITS 32
        
        mov cx, 0x20
        mov ds, cx
        mov es, cx
        mov fs, cx
        mov gs, cx
        mov ss, cx

        add esp, eax

        ret

return_rm:
    BITS 32

    mov eax, [esp + 4]
    sub [esp], eax

    jmp 0x08:.pm16

    .pm16:
        BITS 16

        mov cx, 0x10
        mov ds, cx
        mov es, cx
        mov fs, cx
        mov gs, cx
        mov ss, cx

        sub esp, eax

        shr eax, 4
        mov word [jump_address], .rm
        mov [jump_address + 2], ax
        
        mov eax, cr0
        and eax, ~(1 << 0)
        mov cr0, eax

        jmp far [jump_address]
        
        .rm:
            o32 ret

interrupt_handler:
    BITS 16

    xchg bx, bx
    
    xor eax, eax
    xor ecx, ecx
    mov ax, sp
    mov cx, ss

    mov sp, stack_top
    xor edx, edx
    mov dx, cs
    mov ss, dx
        
    push ds
    push es
    push fs
    push gs

    mov ds, dx
    mov es, dx
    mov fs, dx
    mov gs, dx

    push ax
    push cx

    shl ecx, 4
    shl edx, 4
    add ecx, eax
    
    push ecx
    push edx

    call dword enter_pm32
    BITS 32
    
    mov eax, 0x200000
    call eax
    
    call return_rm
    BITS 16
    
    add sp, 8

    pop cx
    pop ax

    pop gs
    pop fs
    pop es
    pop ds
    
    mov sp, ax
    mov ss, cx
    
    iret

SECTION .data

welcome_message db "LMExtender x86_64 DOS Extender", "$"

jump_address times 6 db 0

ALIGN 8

gdt_start:
    dq 0 ;Null Descriptor

    ;16-Bit Code Descriptor
    dw 0xFFFF
    pm16_code_base times 3 db 0
    db 0b10011011
    db 0b00001111
    db 0

    ;16-Bit Data Descriptor
    dw 0xFFFF
    pm16_data_base times 3 db 0
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
    db 0b00101111
    db 0

    ;64-Bit Data Descriptor
    dw 0xFFFF
    dw 0
    db 0
    db 0b10010011
    db 0b00101111
    db 0

gdtr:
    dw $ - gdt_start - 1
    dd gdt_start

SECTION .bss

stack_bottom:
resb 256
stack_top:

SECTION .upper

start_upper:
INCBIN "lmextender_upper"
end_upper: