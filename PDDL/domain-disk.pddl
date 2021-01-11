;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 1 Op-disk world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain DISKS)
  	(:requirements :strips)
  	(:predicates (DISK ?x) (POLE ?x) 
  				(on ?x ?y) (in ?x ?y) (smaller ?x ?y)
				(handempty) (clear ?x) (holding ?x)
	)

  	(:action pick_last_disk
	     :parameters (?x ?p)
	     :precondition (and (DISK ?x) (POLE ?p) (on ?x ?p) (in ?x ?p) (handempty) (clear ?x))
	     :effect
	     (and (not (handempty))
		   (clear ?p)
		   (holding ?x)
		   (not (clear ?x))
		   (not (on ?x ?p))
		   (not (in ?x ?p))
		 )
	)
	(:action put_in_empty_pole
	     :parameters (?x ?p)
	     :precondition (and (DISK ?x) (POLE ?p) (holding ?x) (clear ?p))
	     :effect
	     (and (not (holding ?x))
		   (not(clear ?p))
		   (handempty)
		   (on ?x ?p)
		   (in ?x ?p)
		   (clear ?x)
		 )
	)
	(:action put_disk
	     :parameters (?x ?p ?y)
	     :precondition (and (DISK ?x) (DISK ?y) (POLE ?p) (holding ?x) (smaller ?x ?y) (in ?y ?p) (clear ?y))
	     :effect
	     (and (not (holding ?x))
		   (handempty)
		   (on ?x ?y)
		   (in ?x ?p)
		   (not (clear ?y))
		   (clear ?x)
		 )
	)
	(:action pick_disk
	     :parameters (?x ?p ?y)
	     :precondition (and (DISK ?x) (DISK ?y) (POLE ?p) (handempty) (on ?x ?y) (in ?y ?p) (in ?x ?p) (clear ?x))
	     :effect
	     (and (not (handempty))
		   (holding ?x)
		   (not (clear ?x))
		   (clear ?y)
		   (not (in ?x ?p))
		   (not(on ?x ?y))
		 )
	)


)
