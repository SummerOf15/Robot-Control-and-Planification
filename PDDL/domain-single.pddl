;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 1 Op-Single world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain SINGLE)
  (:requirements :strips)
(:predicates (MONKEY ?x) (BANANA ?x) (BOX ?x) (ROOM ?x)
  			(is-high ?x) (at ?x ?y) (holding ?x) (is-low ?x)
)

	(:action go
	     :parameters (?x ?y ?z)
	     :precondition (and (MONKEY ?x) (ROOM ?y) (ROOM ?z) (at ?x ?y))
	     :effect
	     (and (not (at ?x ?y))
		   (at ?x ?z)
		)
	)
	(:action push
	     :parameters (?x ?b ?y ?z)
	     :precondition (and (MONKEY ?x) (BOX ?b) (ROOM ?y) (ROOM ?z) (at ?x ?y) (at ?b ?y) (is-low ?x))
	     :effect
	     (and (not (at ?x ?y))
		      (not (at ?b ?y))
		      (at ?x ?z)
			  (at ?b ?z)
		)
	)
	(:action climb
	     :parameters (?x ?b ?y)
	     :precondition (and (MONKEY ?x) (BOX ?b) (ROOM ?y) (is-low ?x) (at ?x ?y) (at ?b ?y))
	     :effect
	     (and (is-high ?x)
		 	  (not(is-low ?x))
		)
	)
	(:action descend
	     :parameters (?x ?b)
	     :precondition (and (MONKEY ?x) (BOX ?b) (is-high ?x))
	     :effect
	     (and (not(is-high ?x))
		)
	)
	(:action grab
	     :parameters (?x ?b ?y ?r)
	     :precondition (and (MONKEY ?x) (BANANA ?b) (ROOM ?r) (BOX ?y) (is-high ?x) (at ?x ?r) (at ?b ?r) (at ?y ?r))
	     :effect
	     (and (holding ?b)
		)
	)
	(:action release
	     :parameters (?x ?b)
	     :precondition (and (MONKEY ?x) (BANANA ?b) (holding ?b))
	     :effect
	     (and (not(holding ?b))
		)
	)	   
)
	