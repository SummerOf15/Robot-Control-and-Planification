;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 1 Op-graph world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain GRAPH)
  (:requirements :strips)
(:predicates (CAR ?x) (NODE ?x)
  			(at ?x ?y)
	       (from-to ?x ?y))

  (:action move
	     :parameters (?x ?y ?z)
	     :precondition (and (CAR ?x) (NODE ?y) (NODE ?z) (from-to ?y ?z) (at ?x ?y))
	     :effect
	     (and (not (at ?x ?y))
		   (at ?x ?z)
		   )))