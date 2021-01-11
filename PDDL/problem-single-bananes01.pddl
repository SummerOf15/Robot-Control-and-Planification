(define (problem Op-Single-01)
(:domain SINGLE)
(:objects monkey banana box
        A B C
)
(:INIT (MONKEY monkey) (BANANA banana) (BOX box)
        (ROOM A) (ROOM B) (ROOM C) (at monkey A) (at banana C) (at box B)
        (is-low monkey) (not(holding banana))
 )
(:goal (AND (holding banana)
        )
))