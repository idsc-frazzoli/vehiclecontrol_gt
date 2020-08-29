# vehiclecontrol_gt

Repository for Game Theory Work related to the IDSC Gokart Lab

Folder Content

1) ForcesKinematicVersion_GameTheory (Developer: Enrico Mion)
2) ForcesMPCPathFollowingDynamic_GameTheory (Developer: Enrico Mion)
3) Manual_ibr (Developer: Thomas Andrews)
4) shared_dynamic (Developer: Enrico Mion)
5) Tire_analysis  (Developer: Enrico Mion)

How to use the folder:
1) Get a Forces Licence and download Forces.

2) Open ForcesKinematicVersion_GameTheory
Folder content:

A) Animation : contains files that draw animation of the vehicles

B) auxiliary_function : contains some auxiliary functions about torque vectoring

C) casadi : auxiliary functions about spline computation

D) c files : files for the go kart

E) constraints : contains the files in which you can define non linear constraints

F) draw_files : contains files to draw simulation results

G) models : contains the models of the optimization problem

H) objective_function : contains the objective functions for the optimization problem

I) parameters_vector : contains the functions that create the parameters vector + parameters script that can be modified by user

J) Run_simulation: Contains Files that runs simulations 

K) index_script: contains scripts about indexing of the different controllers

Compile_controllers.m With this file you can basically run every simulations created with the kynematic model of the gokart. You must set the configuration at the beginning of the file:
NUM_Vehicles: 1,2,3 supported
Compiled: yes or no. Yes to skip compilation phase (Controller must be already compiled)
LEPunisher: yes or no. Cost function option: yes if we want to penalize the lateral error only if the gokart is on the left side of the centerline

For 2 vehicles only:
Condition: 'cen' and dec'. 'cen' means centralized controller (One controller for both), dec means decentralized controller (both agents have their own controller)
Game: 'IBR', 'PG'. Iterated Best Response vs Potential Game. IBR is 'dec' only
Alpha: yes or no. yes if you want consider in PG the following cost function: alpha1*J1+alpha2*J2+Jcol. Only available for 'cen' condition and 'PG'

