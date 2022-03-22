%% Control Selection
% Uncomment one of these to use it.
% The C++ version is more readable, doesn't need globals and is generally
% more pleasant to use.
% the C one is usefull because it's more readilly adapted to most 
% microcontrollers.
% In the subject this is for, C is demanded. The C++ version grew out of the
% frustration of not being able to overload the sum operators in C.

mex controller.cpp lib++/transforms.cpp lib++/control.cpp;
% mex controller.c lib/transforms.c lib/control.c;


%% Constants
Tint = 1e-6;
Ts = 100* Tint;
