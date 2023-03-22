	.syntax unified
	.thumb

	.text
	.global main
	.align
	.thumb_func
	.func main
main:
	// RCC_APB1ENR
	LDR r0, =0x40023840
	LDR r1, [r0]
	ORR r1, r1, #(1<<4) // set bit 4 to enable TIM6 clock
	STR r1, [r0]

 	// TIM6 base address (TIM6 settings start from this address)
	LDR r0, =0x40001000

	// TIM6_PSC: offset 0x28, set prescaler to be 0
	LDR r1, =0x00000000
	STRH r1, [r0, #0x28] // store half word (timer are 16bits) (PSC address 0x40001028)

	// TIM6_ARR: set auto-reload register to be 0xFFFF, you need to find the offset for the ARR register
	LDR r1, =0x0000FFFF
	STRH r1, [r0, #0x2C]

	// TIM6_CR1: offset 0x00
	LDRH r1, [r0, #0] // load half word (load value in [0x40001000] into r1)
	ORR r1, r1, #(1<<0) // set bit0 CEN to enable counter (or the bit0 with 1)
	STRH r1, [r0, #0]   // store back CR1

	// read current value in the counter register TIM6_CNT (offset 0x24): start
	LDRH r1, [r0, #0x24]
	LDR r2, =it1
	STR r1, [r2]  // store it1

    // code you want to measure cycle counts for
    // IMPORTANT: to get accurate cycle count, you can NOT place any break point
    // from here on until where the cycle count is read again.
    // Because when the program is paused, the counter is still counting!!!
	.equ SIZE, 8
	// prepare arguments for function call
	LDR r0, =x
	LDR r1, =y
	LDR r2, =p
	LDR r3, =SIZE

	// call the function
	BL multiply

	// prepare arguments for function call
	LDR r0, =p
	LDR r1, =SIZE

	// call the function
	BL sum

    // save the returned value in r0 to s
	LDR r1, =s
	STR r0, [r1]


	// read current value in the counter register TIM6_CNT (offset 0x24): end
	LDR r0, =0x40001000 // TIM6 base address
	LDRH r1, [r0, #0x24] //blank
	LDR r2, =it2
	STR r1, [r2]  // store it2

	LDR r0, =it1
	LDR r2, [r0]  // read it1

	SUB r1, r1, r2 // calculate cycle count
	LDR r2, =cycle_count
	STR r1, [r2]  // store cycle count

	// TIM6_CR1: offset 0x00
	LDR r0, =0x40001000 // TIM6 base address
	LDRH r1, [r0, #0]
	BIC r1, r1, #(1<<0) // clear bit 0 CEN to disable counter
	STRH r1, [r0, #0]

stop: B stop  // infinite loop

	.endfunc

	.align
	.thumb_func
	.func multiply
multiply:
	PUSH {r4, r5, r6, r7, lr} // preserve registers used in the function call, save return address

	MOV r4, #0  // loop counter
loop0: CMP r4, r3
	BGE exit0  // terminate the loop when r4 >= r3
	LDR r5, [r0, r4, LSL #2] // load element from array 1: r5 = mem(r0 + r4*4), r4 used as offset, int (32-bit) value, so *4
	LDR r6, [r1, r4, LSL #2] // load element from array 2: r6 = mem(r1 + r4*4)
	MUL r7, r5, r6  // multiply
	STR r7, [r2, r4, LSL #2] // store result to array 3: mem(r2 + r4*4) = r7
	ADD r4, r4, #1  // increment loop counter
	B loop0  // all the labels MUST be unique
exit0: POP {r4, r5, r6, r7, pc}  // restore registers, return from function
	.endfunc

	.align
	.thumb_func
	.func sum
sum:
	PUSH {r4, lr} // preserve registers used in the function call, save return address
	MOV r2, #0 // loop counter
	MOV r4, #0 // clear sum
loop1: CMP r2, r1 // terminate the loop when r2 >= r1
	BGE exit1
	LDR r3, [r0], #4 // load array element to r3, post increment r0 by 4
	ADD r4, r4, r3  // sum
	ADD r2, r2, #1  // increment loop counter
	B loop1
exit1: MOV r0, r4   // save result in r0 to return
	POP {r4, pc}  // restore registers, return from function
	.endfunc

	.data
	.align
x: .int 1,1,1,1,1,0,0,0 //memory browser 0x20000000 ~ 0x20000007
y: .int 0,0,0,1,1,1,1,1 //memory browser 0x20000008 ~ 0x2000000F
p: .int 0,0,0,0,0,0,0,0 //memory browser ...
s: .int 0
it1: .int 0
it2: .int 0
cycle_count: .int 0 // memory browser 0x20000063
	.end
