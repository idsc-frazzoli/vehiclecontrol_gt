import numpy
import ctypes

name = "MPCPathFollowing_3v"
requires_callback = True
lib = "lib/MPCPathFollowing_3v.dll"
lib_static = "lib/MPCPathFollowing_3v_static.lib"
c_header = "include/MPCPathFollowing_3v.h"

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (1080,   1), 1080),
 ("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, ( 21,   1),   21),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (3720,   1), 3720)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("alldata"             , ""      , ""               , ctypes.c_double, numpy.float64,     ( 36,), 1080)]

# Info Struct Fields
info = \
[('it', ctypes.c_int),
('it2opt', ctypes.c_int),
('res_eq', ctypes.c_double),
('res_ineq', ctypes.c_double),
('rsnorm', ctypes.c_double),
('rcompnorm', ctypes.c_double),
('pobj', ctypes.c_double),
('dobj', ctypes.c_double),
('dgap', ctypes.c_double),
('rdgap', ctypes.c_double),
('mu', ctypes.c_double),
('mu_aff', ctypes.c_double),
('sigma', ctypes.c_double),
('lsit_aff', ctypes.c_int),
('lsit_cc', ctypes.c_int),
('step_aff', ctypes.c_double),
('step_cc', ctypes.c_double),
('solvetime', ctypes.c_double),
('fevalstime', ctypes.c_double)
]