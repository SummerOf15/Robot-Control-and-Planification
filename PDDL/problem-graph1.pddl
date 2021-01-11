(define (problem Op-graph-01)
(:domain GRAPH)
(:objects car 
        node1 node2 node3 node4 node5
)
(:INIT (CAR car) (NODE node1) (NODE node2) (NODE node3) (NODE node4) (NODE node5)
 (from-to node1 node2) (from-to node2 node3) (from-to node2 node4) (from-to node5 node4) (from-to node3 node5)
 (from-to node1 node3) (from-to node3 node4) (from-to node2 node5)
 (at car node1))
(:goal (AND (at car node5))
))