from cProfile import Profile
import pstats

import boid_py

boid = boid_py.boid(num_boid=200)

def Main():
    
    pr = Profile()

    pr.enable()
    for i in range(60):
        boid.update()
    pr.disable()
    
    stats = pstats.Stats(pr)
    stats.sort_stats('tottime').print_stats()

if __name__ == '__main__':
    Main()