/*
 * asm_func.s
 *
 *  Created on: 7/2/2025
 *      Author: Hou Linxin
 */
   .syntax unified
	.cpu cortex-m4
	.fpu softvfp
	.thumb

		.global asm_func

@ Start of executable code
.section .text

@ CG/[T]EE2028 Assignment 1, Sem 2, AY 2024/25
@ (c) ECE NUS, 2025

@ Write Student 1’s Name here:
@ Lin Huang Ting A0313301A
@ Write Student 2’s Name here:
@ Vu David Hoang A0313181R

@ Look-up table for registers:

@ R0 addr of building
@ R1 addr of entry
@ R2 addr of exit
@ R3 addr of result
@ R4 F value
@ R5 S value
@ R5 loop counter
@ R6 how many car is entering the building
@ R8 immediate value 12
@ R9 how mang car is about to entering (would be modified each loop)
@ R10  building[i], entry[i], exit[i] (reuse of register)

@ write your program from here:
.equ F, 3
.equ S, 2

asm_func:
 	PUSH {R14}

	BL SUBROUTINE

 	POP {R14}

	BX LR

SUBROUTINE:
           LDR R4, =F
           LDR R5, =S
    	   MUL R5, R4, R5
    	   SUB R5, R5, #1
    	   MOV R6, #0

@ calculate how many cars is entering the building
    Entry: LDR R10, [R1], #4
           ADD R6, R6, R10
           SUBS R5, R5, #1
           BNE Entry

           MOV R5, #6 @ reset loop counter
           MOV R10, #0
           MOV R8, #12
@ Park car to each section until it reach maximum capacity or all cars is parked
    Park:  LDR R10, [R0], #4
           SUB R9, R8, R10
           CMP R6, R9

           ITTEE HI
           SUBHI R6, R6, R9
           MOVHI R10, #12
           ADDLS R10, R10, R6
           MOVLS R6, #0
           STR R10, [R3], #4  @ store new existing cars in each section to memory
           SUBS R5, R5, #1
           BNE Park

           SUB R3, R3, #24 @ point to building[0][0]
           MOV R5, #6      @ reset loop counter
@ subtract cars leaving in each section and store it back to memory
	Exit:
		   LDR R10, [R2], #4
		   LDR R8, [R3]
		   SUB R8, R8, R10
		   STR R8, [R3]
		   ADD R3, R3, #4
		   SUBS R5, R5, #1
		   BNE Exit


	      BX LR
