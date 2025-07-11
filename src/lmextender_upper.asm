ORG 0x200000

SECTION .text

%include "lmextender_common.inc"

start_upper:
    BITS 32

    xchg bx, bx
    
    mov eax, [esp + 8]
    
    cmp word [eax + 6], 0
    je .setup

    cmp word [eax + 6], 1
    je .enter_lm

    .setup:
        push edi
        
        mov edi, 0x300000
        xor eax, eax
        mov ecx, 4096
        rep stosd

        mov edi, 0x300000 ;PML4
        mov dword [edi], 0x301000 | 0b11 ;PML3

        mov edi, 0x301000 ;PML3
        mov dword [edi], 0x302000 | 0b11 ;PML2
        
        mov edi, 0x302000 ;PML2
        mov eax, 0x303000 ;PML1
        mov ecx, 8
        
        .fill_pml2:
            mov edx, eax
            or edx, 0b11
            mov dword [edi], eax

            add eax, 0x1000
            add edi, 8

            loop .fill_pml2

        mov edi, 0x303000 ;PML1
        mov edx, 0
        mov ecx, 512 * 8

        .fill_pml1:
            mov eax, edx
            or eax, 0b11
            mov dword [edi], eax

            add edx, 0x1000
            add edi, 8
            
            loop .fill_pml1
        
        pop edi

        ret

    .enter_lm:
        xor ecx, ecx
        xor edx, edx
        
        mov cx, [eax]
        mov dx, [eax + 2]

        shl edx, 4
        add edx, ecx

        pushf
        mov cx, [eax + 4]
        mov [esp], cx
        
        push dword 0x28
        push edx
        
        mov eax, cr4
        or eax, 1 << 5
        mov cr4, eax
    
        mov ecx, 0xC0000080
        rdmsr
        or eax, 1 << 8
        wrmsr

        mov eax, 0x300000
        mov cr3, eax

        mov eax, cr0
        or eax, 1 << 31
        mov cr0, eax

        iret
        
SECTION .data

jump_address times 6 db 0
    
end_upper: