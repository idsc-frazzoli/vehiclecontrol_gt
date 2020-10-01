import numpy
import ctypes

name = "MPCPathFollowing_2v_alpha"
requires_callback = True
lib = "lib/MPCPathFollowing_2v_alpha.dll"
lib_static = "lib/MPCPathFollowing_2v_alpha_static.lib"
c_header = "include/MPCPathFollowing_2v_alpha.h"

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (920,   1),  920),
 ("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (3120,   1), 3120)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("alldata"             , ""      , ""               , ctypes.c_double, numpy.float64,     ( 23,),  920)]

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