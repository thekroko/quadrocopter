#!/usr/bin/python
import os
import sys

def cls():
        os.system(['clear','cls'][os.name == 'nt'])

lastVars = []
for line in sys.stdin: 
  if not line.startswith('data'): continue 
  elems = list(filter(None, line.split('\t')[1:]))[:-1]
  vars = [float(x) for x in elems]
  lastVars.append(vars)
  if len(lastVars) > 256: lastVars.pop(0)
  
  cnt = len(lastVars)
  summed = [sum(x) for x in zip(*lastVars)]
  avg = [x / cnt for x in summed]
  cls()
  print('\t'.join(['Ax', 'Ay', 'Az', 'Gx', 'Gy', 'Gz', 'Mx', 'My', 'Mz', 't', 'p']))
  print('\t'.join([str(round(x)) for x in avg]))
