# Next Steps
6. Implement Optimization interface.
    - 3 objectives 
        a) Leaving the road (>2*n_max) Hard -> If out, end simulation
        b) Smaller final time
        c) Penalize n>n_max. (Going out of the road)
            if (n>nmax): J_t += K*n
    - Choose always the best a). (Longer run).
    - With same a), choose always best c). (Inducing it to return to the road completally)
    - With same a) and c), choose best b).

7. Clean files, comment, make modules.
## LOCAL SEARCH
    - Starting solution:
        a)random
        b) Precalculated actions to folow centerline:
          Fx=Cx*v0^2
          \delta=asin((lR+lF)/R)

    - Neighbour Generation: Modify a parameter applying a normal distribution.
        a) If out of the road before actions being used, only modify the ones before.
            - Modify single action? Action pair: F,delta? (I prefer single change)
            - Choose what action to modify randomly from the electable ones?
            - Apply more importance (more provability to make changes on them) to the previous N actions before going out of the road.
        b) Choose a number of changes Nc, solve in paralell.
            - Not totally in paralell. Make batches of 6 neighbours. (Number of threads)
            - Choose the best in the batch. If it already improves the solution, step finished. Otherwise, continue with next batch.
        c) If path complete. Choose neighbours totally random?
        d) You can combine several "generators", and combine them (Busqueda por entornos variables.)
            - First, you choose the method randomply, but you give more probavility to the methods with greater success.
    - Criterio de parada:
        a) Track completed. (Or time super huge)
         *Not improving time in the last N generations.
## SIMULATED ANNEALING
        
    - Start with this option. Algorithm is very generic. should be just translating from the notes.
        - However, some of the restrictions to electable solutions can be applied as well.
    - But it mtight be slower  than heuristics defined above. Implement a local search algorithm with those heuristics.

    - No pensar en b??squeda tab?? hasta que los dos m??todos anteriores est??n funcionando. Entonces pensar en ella como una heur??stica para guiar las soluciones propuestas por dichos m??todos
"""
Cm: Function to generate starting state
Nm: Function to generate neighbourhood
Sm: Function to select a solution among the neighbourhood
c: Temperature. (Optim step)
L: Number of movements for every c
"""

## Code improvements
    - Usar eval_n.
        - Hacer is_better_sol gen??rico pareto. Input funciones de evaluaci??n. (Por orden de prioridad en caso de dudas).
    - Hacer m??dulos, tests y doc?
