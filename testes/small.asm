.data
vetor: 0,1,2,3

.text 		

main:

	la $t0, vetor           # Load the adress of the vector
	li $t2, 0               # Contador
	li $t3, 3               # Tamanho do vetor
	j print_vec
	
print_vec:
	print $t0               # print vec pointer
	li $t1, 1               
	add $t0, $t1, $t0       # Aumenta o ponteiro do vetor
	add $t2, $t1 $t2        # Aumenta contador
	blti $t2 $t3 print_vec  # Continua até o final do vetor
	
