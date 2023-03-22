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
	ORR r1, r1, #(1<<24) // use memory monitor, note the order of bytes (little endian)
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
    // ===== INSERT YOUR CODE: START =====
    // define SIZE
	.equ SIZE, 6
	// prepare arguments for function call circflip
	LDR r0, =x
	LDR r1, =SIZE

	// call the function circflip
	BL circflip

    // for loop: iterate for SIZE
	mov r8, #0 //
	loop: CMP r8, SIZE
    BGE exit

	
	// prepare arguments for function call multiply
	LDR r0, =x
	LDR r1, =y
	LDR r2, =p
	LDR r3, =SIZE

	// call the function multiply
	BL multiply

	// prepare arguments for function call sum
	LDR r0, =p
	LDR r1, =SIZE

	// call the function sum
	BL sum

	// save the convolution sum
	LDR r1, =s
	ADD r1, r1, r8, LSL#2
	STR r0, [r1]

	// prepare arguments for function call circshift
	LDR r0, =x
	LDR r1, =SIZE

	// call the function circshift
	BL circshift

	ADD r8, r8, #1
	B loop
	// ===== INSERT YOUR CODE: END =====

exit:
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

stop: B stop // infinite loop

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

	.align
	.thumb_func
	.func circflip
circflip:
	PUSH {r4, r5, lr} // preserve registers used in the function call, save return address

	// loop over the array
	MOV r2, #1 //loop index (flip index 1: i)
	SUB r3, r1, #1 //flip index 2: j
loop2: CMP r2, r1 // if i >= length, terminate the loop
	BGE exit2

	// swap
	LDR r4, [r0, r2, LSL #2]
	LDR r5, [r0, r3, LSL #2]
	STR r5, [r0, r2, LSL #2]
	STR r4, [r0, r3, LSL #2]

	// break if i >= j
	CMP r2, r3
	BGE exit2

    // update swap indices
	ADD r2, r2, #1
	SUB r3, r3, #1

	B loop2
exit2: POP {r4, r5, pc} // restore registers, return from function
	.endfunc

	.align
	.thumb_func
	.func circshift
circshift:
	PUSH {r4, r5, lr} // preserve registers used in the function call, save return address

	// loop over the array
	SUB r2, r1, #1 //loop index: start from last element
	LDR r3, [r0, r2, LSL #2] //save the last element

loop3: CMP r2, #0 // if i <= 0, terminate the loop
	BLE exit3

	// circular shift right by 1
	SUB r4, r2, #1
	LDR r5, [r0, r4, LSL #2]
	STR r5, [r0, r2, LSL #2]

    // update loop index
	MOV r2, r4

	B loop3
exit3: STR r3, [r0] // store the last element to be the first element
	POP {r4, r5, pc} // restore registers, return from function
	.endfunc

	.data
	.align

x: .int 1,1,1,1,0,0
y: .int 1,1,1,0,0,0
p: .int 0,0,0,0,0,0
s: .int 0,0,0,0,0,0
//x: .int 1,1,1,1,1,0,0,0
//y: .int 0,0,0,1,1,1,1,1
//p: .int 0,0,0,0,0,0,0,0
//s: .int 0,0,0,0,0,0,0,0
it1: .int 0
it2: .int 0
cycle_count: .int 0
	.end
