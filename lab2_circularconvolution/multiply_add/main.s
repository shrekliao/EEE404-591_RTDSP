	.syntax unified
	.thumb

	.text
	.global main
	.align
	.thumb_func
	.func main
main:
	// enable cycle count
	// enable trace, set DEMCR bit 24
	LDR r0, =0xE000EDFC
	LDR r1, [r0]
	ORR r1, r1, #(1<<24)
	STR r1, [r0]

	// clear DWT_CYCCNT cycle counter
	LDR r0, =0xE0001004
	MOV r1, #0
	STR r1, [r0]

    // set DWT_CTRL bit 0 to enable CYCCNT register
	LDR r0, =0xE0001000
	LDR r1, [r0]
	ORR r1, r1, #(1<<0)
	STR r1, [r0]

	// read current cycle count: start
	LDR r0, =0xE0001004
	LDR r1, [r0]
	LDR r2, =it1
	STR r1, [r2]  // store it1

    // code you want to measure cycle counts for
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

	// read current cycle count: end
	LDR r0, =0xE0001004
	LDR r1, [r0]
	LDR r2, =it2
	STR r1, [r2]  // store it2

	LDR r0, =it1
	LDR r2, [r0]  // read it1

	SUB r1, r1, r2 // calculate cycle count
	LDR r2, =cycle_count
	STR r1, [r2]  // store cycle count

	// clear DWT_CTRL bit 0 to disable CYCCNT register
	LDR r0, =0xE0001000
	LDR r1, [r0]
	BIC r1, r1, #(1<<0)
	STR r1, [r0]

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
x: .int 1,1,1,1,1,0,0,0
y: .int 0,0,0,1,1,1,1,1
p: .int 0,0,0,0,0,0,0,0
s: .int 0
it1: .int 0
it2: .int 0
cycle_count: .int 0
	.end
