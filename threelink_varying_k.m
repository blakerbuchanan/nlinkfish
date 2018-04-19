(* ::Package:: *)

(* ::Title::Initialization:: *)
(*Three-link Swimmer with Varying Stiffness and Amplitude*)


(* ::Text::Initialization:: *)
(*The obvious thing to do is start by reproducing the three-link using Scott's notebook as a guide.*)


(* ::Input::Initialization:: *)
robotParameters = {a->0.0725, b->0.15, gap-> 0.02};


(* ::Input::Initialization:: *)
zbody = {x[t], y[t]};


(* ::Input::Initialization:: *)
zhead= zbody+{(b+gap)Cos[\[Theta][t]]+(b+gap)Cos[\[Theta][t]+\[Phi][t]],(b+gap)Sin[\[Theta][t]]+(b+gap)Sin[\[Theta][t]+\[Phi][t]]};


(* ::Input::Initialization:: *)
ztail = zbody-{(b+gap)Cos[\[Theta][t]]+(b+gap)Cos[-\[Theta]1[t]+\[Theta][t]],(b+gap)Sin[\[Theta][t]]+(b+gap)Sin[-\[Theta]1[t]+\[Theta][t]]};


(* ::Input::Initialization:: *)
headPivot = zbody+{(b+gap)Cos[\[Theta][t]],(b+gap)Sin[\[Theta][t]]};


(* ::Input::Initialization:: *)
tailPivot = zbody-{(b+gap)Cos[\[Theta][t]],(b+gap)Sin[\[Theta][t]]};


(* ::Input::Initialization:: *)
body = {Purple,Opacity[0.4],EdgeForm[Thin],Disk[zbody,{b,a}]}//Rotate[#,\[Theta][t],zbody]&//Graphics;


(* ::Input::Initialization:: *)
headPivotDot=Disk[headPivot,0.02]//Graphics;


(* ::Input::Initialization:: *)
headPivotLine1=Line[{zbody,headPivot}]//Graphics;


(* ::Input::Initialization:: *)
headPivotLine2 = Line[{headPivot,zhead}]//Graphics;


(* ::Input::Initialization:: *)
(*head = Circle[zhead,{b,a}]//Rotate[#, \[Theta][t]+\[Phi][t]]&//Graphics;*)


(* ::Input::Initialization:: *)
head = {Purple,Opacity[0.4],EdgeForm[Thin],Disk[zhead,{b,a}]}//Rotate[#, \[Theta][t]+\[Phi][t]]&//Graphics;


(* ::Input::Initialization:: *)
tailPivotDot = Disk[tailPivot, 0.02]//Graphics;


(* ::Input::Initialization:: *)
tailPivotLine1 = Line[{zbody,tailPivot}]//Graphics;


(* ::Input::Initialization:: *)
tailPivotLine2 = Line[{ztail,tailPivot}]//Graphics;


(* ::Input::Initialization:: *)
tail = {Purple,Opacity[0.4],EdgeForm[Thin],Disk[ztail,{b,a}]}//Rotate[#,\[Theta][t]-\[Theta]1[t]]&//Graphics;


(* ::Input::Initialization:: *)
water = {Blue,Opacity[0.7],Rectangle[{-8 ,8}, {-8 , 8}]}//Graphics;


(* ::Input::Initialization:: *)
Manipulate[Show[{body,headPivotDot,head,tail,tailPivotDot,headPivotLine1,headPivotLine2,tailPivotLine1,tailPivotLine2}/.robotParameters/.{x[t]-> X, y[t]-> Y,\[Theta][t]-> \[CapitalTheta],\[Theta]1[t]-> \[CapitalTheta]1,\[Phi][t]-> \[CapitalPhi]},PlotRange-> {{-(3 b + gap + 2) , 3 b + gap + 2}, {-(3 b + gap + 2) , 3 b+ gap + 2}} /. robotParameters],{X,-1,1},{Y,-1,1},{\[CapitalTheta],0,2\[Pi]},{\[CapitalTheta]1,-\[Pi]/2,\[Pi]/2},{\[CapitalPhi],-\[Pi]/2,\[Pi]/2}];


(* ::Chapter::Initialization:: *)
(*Three-link System Dynamics*)


(* ::Input::Initialization:: *)
ebodylong={Cos[\[Theta][t]],Sin[\[Theta][t]]};


(* ::Input::Initialization:: *)
ebodylat = {-Sin[\[Theta][t]], Cos[\[Theta][t]]};


(* ::Input::Initialization:: *)
eheadlong={Cos[\[Theta][t]+\[Phi][t]],Sin[\[Theta][t]+\[Phi][t]]};


(* ::Input::Initialization:: *)
eheadlat = {-Sin[\[Theta][t]+\[Phi][t]],Cos[\[Theta][t]+\[Phi][t]]};


(* ::Input::Initialization:: *)
etaillong0 = {Cos[\[Theta]1[t]-\[Theta][t]], Sin[\[Theta]1[t]-\[Theta][t]]};
etaillong = {Cos[-\[Theta]1[t]+\[Theta][t]], Sin[-\[Theta]1[t]+\[Theta][t]]};


(* ::Input::Initialization:: *)
etaillat0 = {-Sin[\[Theta]1[t]-\[Theta][t]], Cos[\[Theta]1[t]-\[Theta][t]]};
etaillat = {-Sin[-\[Theta]1[t]+\[Theta][t]], Cos[-\[Theta]1[t]+\[Theta][t]]};


(* ::Input::Initialization:: *)
ellipseMasses = {mLong-> \[Rho] \[Pi] a^2,mLat -> \[Rho] \[Pi] b^2, mRot -> \[Rho] \[Pi]/8 (b^2-a^2)^2};


(* ::Input::Initialization:: *)
vhead=D[zhead,t];


(* ::Input::Initialization:: *)
vbody=D[zbody, t];


(* ::Input::Initialization:: *)
vtail=D[ztail,t];


(* ::Input::Initialization:: *)
KEbody = 1/2 mLong (vbody.ebodylong)^2+1/2 mLat (vbody.ebodylat)^2+1/2 mRot (\[Theta]'[t])^2;


(* ::Input::Initialization:: *)
KEhead= 1/2 mLong (vhead.eheadlong)^2+1/2 mLat (vhead.eheadlat)^2+1/2 mRot (\[Theta]'[t]+\[Phi]'[t])^2;


(* ::Input::Initialization:: *)
KEtail= 1/2 mLong (vtail.etaillong)^2 + 1/2 mLat (vtail.etaillat)^2+1/2 mRot (\[Theta]'[t]-\[Theta]1'[t])^2;


(* ::Input::Initialization:: *)
PEtail = 1/2 k1 \[Theta]1[t]^2;


(* ::Text::Initialization:: *)
(*What are the units of k? N*m/rad, I think...*)


(* ::Input::Initialization:: *)
L = KEbody+KEhead+KEtail-PEtail;


(* ::Input::Initialization:: *)
liftedAction = {x[t]-> xbar + x[t] Cos[\[Theta]bar]-y[t]Sin[\[Theta]bar],y[t]-> ybar+x[t] Sin[\[Theta]bar]+y[t] Cos[\[Theta]bar],\[Theta][t]-> \[Theta][t]+\[Theta]bar,x'[t]-> x'[t] Cos[\[Theta]bar]-y'[t] Sin[\[Theta]bar],y'[t]-> x'[t] Sin[\[Theta]bar]+y'[t] Cos[\[Theta]bar]};


(* ::Input::Initialization:: *)
L - L/.liftedAction


(* ::Text::Initialization:: *)
(*Define the infinitesimal generators. Let me make sure that I conceptually understand the infinitesimal generator. If \[CapitalPhi] acts on elements of G x Q and results in "motion" in Q, Subscript[\[Zeta], Q] (the infinitesimal generator) is a vector field that takes as arguments elements of Q and gives tangent vectors in Q, according to \!\( *)
(*\*FractionBox[\(d\), \(d\[Epsilon]\)]*)
(*\*SubscriptBox[\(|\), \(\[Epsilon] = 0\)]\((\[CapitalPhi](exp(\[Epsilon]\ \[Zeta]), q)\)\). It isn't entirely clear why we need to define two Lie algebra elements. Oh, wait. A Lie algebra element is needed for each joint motion.*)


(* ::Input::Initialization:: *)
\[Zeta]={\[Zeta]1, \[Zeta]2, \[Zeta]3};


(* ::Input::Initialization:: *)
\[Zeta]Q = {\[Zeta]1 - y[t] \[Zeta]3, \[Zeta]2 + x[t] \[Zeta]3, \[Zeta]3, 0, 0};


(* ::Input::Initialization:: *)
\[Lambda] = {\[Lambda]1, \[Lambda]2, \[Lambda]3};


(* ::Input::Initialization:: *)
\[Lambda]Q = {\[Lambda]1 - y[t] \[Lambda]3, \[Lambda]2 + x[t] \[Lambda]3, \[Lambda]3, 0, 0};


(* ::Text::Initialization:: *)
(*Since there is an SE(2) symmetry, the momentum maps associated with this symmetry are obtained by the natural pairing of the infinitesimal generators with the fiber derivative of the Lagrangian. (Skip to where we define the momentum maps. The following code just defines the ODEs for the three link swimmer.)*)


(* ::Input::Initialization:: *)
xode = D[D[L,x'[t]],t]-D[L,x[t]]==0;


(* ::Input::Initialization:: *)
yode = D[D[L,y'[t]],t]-D[L,y[t]]==0;


(* ::Input::Initialization:: *)
\[Theta]ode = D[D[L,\[Theta]'[t]],t]-D[L,\[Theta][t]]==0;


(* ::Input::Initialization:: *)
\[Theta]1ode = D[D[L,\[Theta]1'[t]],t]-D[L,\[Theta]1[t]]==-c1 \[Theta]1'[t];


(* ::Input::Initialization:: *)
odes ={xode,yode,\[Theta]ode,\[Theta]1ode};


(* ::Input::Initialization:: *)
freq = 0.2;


(* ::Input::Initialization:: *)
omega = 2 3.14159 freq;


(* ::Input::Initialization:: *)
A=1;


(* ::Input::Initialization:: *)
shapeChange = {\[Phi][t]-> A Cos[2.0 \[Pi] freq t]};
fluidAndSpringParams = {k1-> 1.0,\[Rho]-> 1000};
dampingParams={c1-> 2 Sqrt[k1 mRot]};


(* ::Text::Initialization:: *)
(*How can I always ensure the system is critically (or close to) damped for simulation?*)


(* ::Input::Initialization:: *)
numofcycles = 10;


(* ::Input::Initialization:: *)
duration = numofcycles/freq;


(* ::Input::Initialization:: *)
solutionControl = NDSolve[{odes/.dampingParams/.ellipseMasses/.fluidAndSpringParams/.robotParameters/.shapeChange/.D[shapeChange,t]/.D[D[shapeChange,t],t],x[0]==0,y[0]==0,\[Theta][0]==0,x'[0]==0,y'[0]==0,\[Theta]'[0]==0,\[Theta]1[0]==0,\[Theta]1'[0]==0},{x[t],y[t],\[Theta][t],x'[t],y'[t],\[Theta]'[t],\[Theta]1[t],\[Theta]1'[t]},{t,0,duration},Method->{"EquationSimplification"-> "Residual"}];


(* ::Input::Initialization:: *)
Plot[Evaluate[{x[t], y[t], \[Theta][t]} /. solutionControl], {t, 0, duration},PlotLegends->{x[t],y[t],\[Theta][t]}];


(* ::Input::Initialization:: *)
movieFrame[time_]:=Show[{body,head,tail,tailPivotLine1,tailPivotDot,tailPivotLine2,headPivotDot,headPivotLine1,headPivotLine2}/.robotParameters/.shapeChange/.solutionControl/.t-> time,PlotRange-> {{-(6 b + gap + 2) , 6 b + gap + 2}, {-(6 b + gap + 2) , 6 b + gap + 2}} /. robotParameters];


(* ::Input::Initialization:: *)
numberOfFrames = 300;


(* ::Input::Initialization:: *)
movieFrames = Table[movieFrame[k],{k,0,duration,duration/numberOfFrames}];


(* ::Input::Initialization:: *)
ListAnimate[movieFrames];


(* ::Input::Initialization:: *)
FL = {D[L,x'[t]],D[L,y'[t]],D[L,\[Theta]'[t]],D[L,\[Phi]'[t]],D[L,\[Theta]1'[t]]};


(* ::Text::Initialization:: *)
(*We define the momentum map by <J(Subscript[v, q]),\[Eta]> which equals <FL(Subscript[v, q]),Subscript[\[Eta], Q](q)>*)


(* ::Input::Initialization:: *)
J1 = FL.\[Zeta]Q/\[Zeta]1/.{\[Zeta]2-> 0,\[Zeta]3-> 0};


(* ::Input::Initialization:: *)
J2 = FL.\[Zeta]Q/\[Zeta]2/.{\[Zeta]1-> 0,\[Zeta]3-> 0};


(* ::Input::Initialization:: *)
J3 = FL.\[Zeta]Q/\[Zeta]3/.{\[Zeta]2-> 0,\[Zeta]1-> 0}//FullSimplify;


(* ::Text::Initialization:: *)
(*Everything so far is commensurate with Scott's analysis of a similar 3-link swimmer. Let's get to the good stuff. We will, as per Scott's analysis, need the locked inertia tensor, the connection form, and the local connection form.*)


(* ::Input::Initialization:: *)
rightpairing = \[Lambda]Q.FL/.{x'[t]-> \[Zeta]Q[[1]],y'[t]-> \[Zeta]Q[[2]],\[Theta]'[t]-> \[Zeta]Q[[3]],\[Phi]'[t]-> \[Zeta]Q[[4]],\[Theta]1'[t]-> \[Zeta]Q[[5]]};


(* ::Input::Initialization:: *)
lockedInertiaTensor = FullSimplify[{{(rightpairing/.{\[Zeta]2-> 0,\[Zeta]3-> 0,\[Lambda]2-> 0,\[Lambda]3-> 0})/(\[Zeta]1 \[Lambda]1),(rightpairing/.{\[Zeta]1-> 0,\[Zeta]3-> 0,\[Lambda]2-> 0,\[Lambda]3-> 0})/(\[Zeta]2 \[Lambda]1),(rightpairing/.{\[Zeta]1-> 0,\[Zeta]2-> 0,\[Lambda]2-> 0,\[Lambda]3-> 0})/(\[Zeta]3 \[Lambda]1)},{(rightpairing/.{\[Zeta]2-> 0,\[Zeta]3-> 0,\[Lambda]1-> 0,\[Lambda]3-> 0})/(\[Zeta]1 \[Lambda]2),(rightpairing/.{\[Zeta]1-> 0,\[Zeta]3-> 0,\[Lambda]1-> 0,\[Lambda]3-> 0})/(\[Zeta]2 \[Lambda]2),(rightpairing/.{\[Zeta]1-> 0,\[Zeta]2-> 0,\[Lambda]1-> 0,\[Lambda]3-> 0})/(\[Zeta]3 \[Lambda]2)},{(rightpairing/.{\[Zeta]2-> 0,\[Zeta]3-> 0,\[Lambda]2-> 0,\[Lambda]1-> 0})/(\[Zeta]1 \[Lambda]3),(rightpairing/.{\[Zeta]1-> 0,\[Zeta]3-> 0,\[Lambda]2-> 0,\[Lambda]1-> 0})/(\[Zeta]2 \[Lambda]3),(rightpairing/.{\[Zeta]1-> 0,\[Zeta]2-> 0,\[Lambda]1-> 0,\[Lambda]2-> 0})/(\[Zeta]3 \[Lambda]3)}}];


(* ::Input::Initialization:: *)
\[CapitalGamma]=Inverse[lockedInertiaTensor].{J1,J2,J3};


(* ::Input::Initialization:: *)
Adg = {{Cos[\[Theta][t]],-Sin[\[Theta][t]],y[t]},{Sin[\[Theta][t]],Cos[\[Theta][t]],-x[t]},{0,0,1}};


(* ::Input::Initialization:: *)
localLIT = Transpose[Adg].lockedInertiaTensor.Adg;


(* ::Text::Initialization:: *)
(*The following verification checks out when the joint angles are zero. Huzzah!*)


(* ::Input::Initialization:: *)
localLIT/.{\[Phi][t]-> 0,\[Theta]1[t]-> 0}//Simplify//MatrixForm;


(* ::Input::Initialization:: *)
SE2BV = {x'[t] Cos[\[Theta][t]] + y'[t] Sin[\[Theta][t]], y'[t] Cos[\[Theta][t]] -x'[t] Sin[\[Theta][t]], \[Theta]'[t]};


(* ::Text::Initialization:: *)
(*A mechanical connection is given by \[CapitalGamma]=A(r) rdot + g^-1 gdot=A(r) rdot + SE2BV*)


(* ::Input::Initialization:: *)
g = {{Cos[\[Theta][t]],-Sin[\[Theta][t]], x[t]},{Sin[\[Theta][t]],Cos[\[Theta][t]], y[t]},{0,0,1}};


(* ::Input::Initialization:: *)
Inverse[g].D[g,t]//FullSimplify//MatrixForm;


(* ::Text::Initialization:: *)
(*Okay, great. Just wanted to check my understanding of how we represent the body velocity. I now recall that we represent the above matrix as a 3x1 vector.*)


(* ::Input::Initialization:: *)
A = Inverse[Adg].\[CapitalGamma] - SE2BV;


(* ::Text::Initialization:: *)
(*Much like Scott's approach to finding the local connection form, I will not wait for Mathematica to simplify A. How can I exploit what I know A should look like so as to require less from Mathematica computationally?*)


(* ::Input::Initialization:: *)
trickA=\[CapitalGamma]/.{x[t]-> 0, y[t]-> 0,\[Theta][t]-> 0,x'[t]-> 0,y'[t]-> 0,\[Theta]'[t]-> 0};


(* ::Input::Initialization:: *)
Ax\[Phi]=trickA[[1]]/.{\[Phi]'[t]-> 1,\[Theta]1'[t]-> 0};


(* ::Input::Initialization:: *)
Ax\[Theta]1=trickA[[1]]/.{\[Phi]'[t]-> 0,\[Theta]1'[t]-> 1};


(* ::Input::Initialization:: *)
Ay\[Phi]=trickA[[2]]/.{\[Phi]'[t]-> 1,\[Theta]1'[t]-> 0};


(* ::Input::Initialization:: *)
Ay\[Theta]1=trickA[[2]]/.{\[Phi]'[t]-> 0,\[Theta]1'[t]-> 1};


(* ::Input::Initialization:: *)
A\[Theta]\[Phi]=trickA[[3]]/.{\[Phi]'[t]-> 1,\[Theta]1'[t]-> 0};


(* ::Input::Initialization:: *)
A\[Theta]\[Theta]1=trickA[[3]]/.{\[Phi]'[t]-> 0,\[Theta]1'[t]-> 1};


(* ::Input::Initialization:: *)
fiberODE={SE2BV[[1]]==-trickA[[1]],SE2BV[[2]]==-trickA[[2]],SE2BV[[3]]==-trickA[[3]]};

Save["fiberODE",fiberODE];

(* ::Chapter::Initialization:: *)
(*Q-learning for Varying Stiffness and Head Speed*)


(* ::Text::Initialization:: *)
(*We want to determine the optimal way to vary the stiffness of a spring in the three-link swimmer over some specified number of cycles. This seems like a good starting point. We can then add the amplitude variation after successfully optimizing the spring stiffness.*)


(* ::Input::Initialization:: *)
PEtailk = 1/2 k1 \[Theta]1[t]^2;


(* ::Input::Initialization:: *)
Lk = KEbody+KEhead+KEtail-PEtailk;


(* ::Input::Initialization:: *)
xodek = D[D[Lk,x'[t]],t]-D[Lk,x[t]]==0;


(* ::Input::Initialization:: *)
yodek = D[D[Lk,y'[t]],t]-D[Lk,y[t]]==0;


(* ::Input::Initialization:: *)
\[Theta]odek = D[D[Lk,\[Theta]'[t]],t]-D[Lk,\[Theta][t]]==0;


(* ::Input::Initialization:: *)
\[Theta]1odek = D[D[Lk,\[Theta]1'[t]],t]-D[Lk,\[Theta]1[t]]==-c1 \[Theta]1'[t];


(* ::Input::Initialization:: *)
\[Phi]odek = D[D[Lk,\[Phi]'[t]],t]-D[Lk,\[Phi][t]]==0;


(* ::Input::Initialization:: *)
odesk ={xodek,yodek,\[Theta]odek,\[Theta]1odek};


(* ::Text::Initialization:: *)
(*PSEUDOCODE*)
(**)
(*Establish Q matrix, states, and actions.*)
(*Define terminal state*)
(**)
(*Start a loop:*)
(*	Initialize state*)
(*	Start another loop:*)
(*		Choose the action based on the policy*)
(*		Take action, observe reward and next state*)
(*		Update Q function*)
(*		Update state*)
(*	until S is terminal*)


(* ::Text::Initialization:: *)
(*Define learning rate, discount factor, time step*)


(* ::Input::Initialization:: *)
\[Alpha]=0.9;


(* ::Input::Initialization:: *)
\[Gamma]=0.95;


(* ::Input::Initialization:: *)
\[Delta]t=1;


(* ::Input::Initialization:: *)
eps=0.3;


(* ::Input::Initialization:: *)
Reward[currentSol_,pastSol_]:=currentSol-pastSol;


(* ::Input::Initialization:: *)
QUpdate[newPos_,oldPos_,actionPos_,reward_,Q_]:=Q[[oldPos[[1]],oldPos[[2]],oldPos[[3]],oldPos[[4]],actionPos[[1]],actionPos[[2]]]]+\[Alpha] (reward+\[Gamma] Max[ Q[[newPos[[1]],newPos[[2]],newPos[[3]],newPos[[4]]]]]-Q[[oldPos[[1]],oldPos[[2]],oldPos[[3]],oldPos[[4]],actionPos[[1]],actionPos[[2]]]]);


(* ::Input::Initialization:: *)
getActionPos[action_]:={Position[stiffnessAction,action[[1]]][[1]][[1]],Position[\[Phi]dotAction,action[[2]]][[1]][[1]]};


(* ::Text::Initialization:: *)
(*We could define an epsilon that decays as the number of episodes increases.*)


(* ::Input::Initialization:: *)
getMaxPos[state_,Q_]:=Position[Q[[state[[1]],state[[2]],state[[3]],state[[4]]]],Max[Q[[state[[1]],state[[2]],state[[3]],state[[4]]]]]][[1]];


(* ::Input::Initialization:: *)
checkPhiStateRand[phi_,phiBounds_,actionBound_]:=If[MemberQ[phiBounds,phi]== True,return=RandomChoice[{0.,phi (Abs[phi])^-1 actionBound}],return=RandomChoice[\[Phi]dotAction]];


(* ::Input::Initialization:: *)
(*chooseAction[eps_,state_,Q_,phi_,phiBounds_,actionBound_]:= If[eps>Random[],action={RandomChoice[stiffnessAction],checkPhiStateRand[phi,phiBounds,actionBound]},action={stiffnessAction[[getMaxPos[state,Q][[1]]]],If[MemberQ[phiBounds,phi]\[Equal] True,action=RandomChoice[{0.,phi (Abs[phi])^-1actionBound}],action=\[Phi]dotAction[[getMaxPos[state,Q][[2]]]]]}];*)


(* ::Input::Initialization:: *)
chooseAction[eps_,state_,Q_]:= If[eps>Random[],action={RandomChoice[stiffnessAction],RandomChoice[\[Phi]dotAction]},action={stiffnessAction[[getMaxPos[state,Q][[1]]]],action=\[Phi]dotAction[[getMaxPos[state,Q][[2]]]]}];


(* ::Input::Initialization:: *)
\[Phi]Update[\[Phi]0_,t_,\[Phi]dot_]:=Mod[\[Phi]0+Integrate[\[Phi]dot,t],2 \[Pi]];


(* ::Input::Initialization:: *)
takeAction[action_,initialCon_,time_,\[Phi]0_]:=NDSolve[{odesk/.{c1-> 1}/.ellipseMasses/.{k1-> action[[1]]}/.{\[Phi][t]-> \[Phi]0+action[[2]] \[Delta]t}/.{\[Phi]'[t]-> action[[2]]}/.{\[Phi]''[t]-> 0.0}/.{\[Rho]-> 1000}/.robotParameters,initialCon},{x[t],y[t],\[Theta][t],x'[t],y'[t],\[Theta]'[t],\[Theta]1[t],\[Theta]1'[t]},{t,time, time+\[Delta]t},Method->{"EquationSimplification"-> "Residual"},InterpolationOrder->All];


(* ::Input::Initialization:: *)
takeActionReduced[action_,initialCon_,time_,\[Phi]0_]:=NDSolve[{{fiberODE,\[Theta]1odek}/.{c1-> 1}/.ellipseMasses/.{k1->action[[1]]}/.{\[Phi][t]-> \[Phi]0+action[[2]] \[Delta]t}/.{\[Phi]'[t]-> action[[2]]}/.{\[Phi]''[t]-> 0.0}/.{\[Rho]-> 1000}/.robotParameters,initialCon},{x[t],y[t],\[Theta][t],x'[t],y'[t],\[Theta]'[t],\[Theta]1[t],\[Theta]1'[t]},{t,time, time+\[Delta]t},Method->{"EquationSimplification"-> "Residual"},InterpolationOrder->All];


(* ::Input::Initialization:: *)
numOfEpisodes = 1;


(* ::Input::Initialization:: *)
lengthOfEpisode=20;


(* ::Input::Initialization:: *)
exportCounter=numOfEpisodes/1000;


(* ::Input::Initialization:: *)
getStatePos[state_]:={Position[states,state[[1]]][[1]],Position[states,state[[2]]][[1]],Position[states,state[[3]]][[1]],Position[states,state[[4]]][[1]]};


(* ::Input::Initialization:: *)
getClosestStatePos[states_,newState_]:={Position[states[[1]],Nearest[states[[1]],newState[[1]]][[1]]][[1,1]],Position[states[[2]],Nearest[states[[2]],newState[[2]]][[1]]][[1,1]],Position[states[[3]],Nearest[states[[3]],newState[[3]]][[1]]][[1,1]],Position[states[[4]],Nearest[states[[4]],newState[[4]]][[1]]][[1,1]]};


(* ::Input::Initialization:: *)
getInitialPos[state1_,state2_,state3_,state4_]:={Position[state1,0.][[1]][[1]],Position[state2,0.][[1]][[1]],Position[state3,0][[1]][[1]],Position[state4,0.][[1]][[1]]};


(* ::Input::Initialization:: *)
currentTimeAndDate=DateList[];


(* ::Input::Initialization:: *)
timeAndDateStrings=(ToString[#]&/@currentTimeAndDate);


(* ::Input::Initialization:: *)
space=ToString[_];


(* ::Input::Initialization:: *)
timeAndDateString=timeAndDateStrings[[1]]<>space<>timeAndDateStrings[[2]]<>space<>timeAndDateStrings[[3]]<>space<>timeAndDateStrings[[4]]<>space<>timeAndDateStrings[[5]];


(* ::Input::Initialization:: *)
(*Initialize Q matrix, states, actions, and terminal state (defined as the maximum time one can wander through the state space)*)
begintime=AbsoluteTime[];
phiRange=0.8;
\[Theta]states = Range[0.0,2\[Pi],\[Pi]/10];
\[Theta]1states=Range[0.0,\[Pi],\[Pi]/10];
\[Phi]states=Range[0.0,\[Pi],\[Pi]/10];
\[Theta]1dotstates=Range[-\[Pi]/2,\[Pi]/2,\[Pi]/10];
\[Phi]dotAction=Range[-phiRange,phiRange,phiRange];
stiffnessAction=Range[0.1,0.5,1/10];
stopForExport=1000;
states={\[Theta]states,\[Theta]1states,\[Theta]1dotstates,\[Phi]states};
Q=Table[0.0,{s1,1,Length[\[Theta]states]},{s2,1,Length[\[Theta]1states]},{s3,1,Length[\[Theta]1dotstates]},{s4,1,Length[\[Phi]states]},{a1,1,Length[stiffnessAction]},{a2,1,Length[\[Phi]dotAction]}];
(*Run a series of episodes*)
For[i=1, i<= numOfEpisodes, i++,initialConditions={x[0]==0,y[0]==0,\[Theta][0]==0,\[Theta]1[0]==0,\[Theta]1'[0]==0};
(*Initialize*)
statePos=getInitialPos[\[Theta]states,\[Theta]1states,\[Theta]1dotstates,\[Phi]states]; pastx=0;\[Phi]0=0.0;
(*Take "lengthOfEpisode" steps in the environment*)
For[j=0, j<lengthOfEpisode, j++,
(*Choose the action from current state using desired policy*)
action=chooseAction[eps,statePos,Q];actionPos=getActionPos[action];
(*Print[action];*)
 (*Take action and observe distance in x direction (reward)*)
solution=takeActionReduced[action,initialConditions,j,\[Phi]0];
newx=((x[t]/.solution)/.{t->(j+\[Delta]t)})[[1]];
(*Observe new state*)
newState=(({\[Theta][t],\[Theta]1[t],\[Theta]1'[t],\[Phi]0+action[[2]] \[Delta]t}/.solution)/.{t->j+\[Delta]t})[[1]];
\[Phi]0=newState[[4]];
(*Use Mod[anglestates,2pi]*)
newState={Mod[newState[[1]],2 \[Pi]],Mod[newState[[2]],2 \[Pi]],newState[[3]],Mod[newState[[4]],2 \[Pi]]};
(*Determine which discretized state newState is closest to. Really this finds the closest position in the set of states and returns it so it can be matched in the Q matrix.*)
newStatePos=getClosestStatePos[states,newState];
(*Compute reward and update previous x position*)
r=Reward[newx,pastx];
pastx=newx;
(*Update Q matrix*)
Q[[statePos[[1]],statePos[[2]],statePos[[3]],statePos[[4]],actionPos[[1]],actionPos[[2]]]]=QUpdate[newStatePos,statePos,actionPos,r,Q];
statePos=newStatePos;
initialConditions=(({x[j+\[Delta]t]==x[t],y[j+\[Delta]t]==y[t],\[Theta][j+\[Delta]t]== \[Theta][t],\[Theta]1[j+\[Delta]t]==\[Theta]1[t],\[Theta]1'[j+\[Delta]t]==\[Theta]1'[t]}/.solution)/.{t->j+\[Delta]t})[[1]];
If[i==stopForExport,Save["Qmatrix_"<>timeAndDateString<>"_N"<>ToString[i],Q];stopForExport=stopForExport+exportCounter,0]]];
endtime=AbsoluteTime[];


(* ::Input::Initialization:: *)
totaltime=endtime-begintime


(* ::Input::Initialization:: *)
totaltime/3600


(* ::Input::Initialization:: *)
Qfinal=Q;


(* ::Subchapter::Initialization:: *)
(*Policy Extraction and Simulation*)


(* ::Input::Initialization:: *)
statePos=getInitialPos[\[Theta]states,\[Theta]1states,\[Theta]1dotstates,\[Phi]states];
initialConditions={x[0]==0,y[0]==0,\[Theta][0]==0,\[Theta]1[0]==0,\[Theta]1'[0]==0};
\[Phi]0=0.0;
newState==\[Phi]0;
listOfSol={};
listOfPlots={};
listOfActionPlots={};
listOfPhiPlots={};
listOfPhiDotPlots={};
listOfActions={};
listOfMomentumPlots={};
For[time=0,time<lengthOfEpisode,time++,
(*Use an epsilon of zero in the chooseAction function (act entirely greedy)*)
action=chooseAction[0.0,statePos,Qfinal];
AppendTo[listOfActions,action];
solutionFinal=takeActionReduced[action,initialConditions,time,\[Phi]0];
newState=(({\[Theta][t],\[Theta]1[t],\[Theta]1'[t],\[Phi]0+action[[2]] \[Delta]t}/.solutionFinal)/.{t-> time+\[Delta]t})[[1]];
\[Phi]0=newState[[4]];
(*Use Mod[anglestates,2pi]*)
newState={Mod[newState[[1]],2 \[Pi]],Mod[newState[[2]],2\[Pi]],newState[[3]],Mod[newState[[4]],2 \[Pi]]};
(*Print[action[[2]]];
Print[newState[[4]]];*)
(*Determine which discretized state newState is closest to.*)
newStatePos=getClosestStatePos[states,newState];
statePos=newStatePos;
(*Generate a plot for the state and action over \[Delta]t*)
AppendTo[listOfSol,solutionFinal];
If[time==0,plotState=Plot[Evaluate[{x[t], y[t], \[Theta][t]}/.solutionFinal], {t,time,time+\[Delta]t},PlotLegends-> {"x[t] (m)","y[t] (m)","\[Theta][t] (radians)"},PlotStyle->{Blue,Green,Red}],plotState=Plot[Evaluate[{x[t], y[t], \[Theta][t]} /.solutionFinal], {t,time,time+\[Delta]t},PlotStyle->{Blue,Green,Red}]];
AppendTo[listOfPlots,plotState];
If[time==0,plotAction=Plot[action,{t,time,time+\[Delta]t}, PlotLegends->{"k1[t] (Nms/(radian))","\[Phi]'[t] (radians)"},PlotStyle->{Cyan,Magenta}],plotAction=Plot[action, {t, time, time+\[Delta]t},PlotStyle->{Cyan,Magenta}]];
AppendTo[listOfActionPlots,plotAction];
(*Compute the momenta*)
momentum={J1,J2,J3}/.ellipseMasses/.{k1-> action[[1]]}/.{\[Rho]->1000}/.robotParameters/.{\[Phi][t]->\[Phi]0}/.{\[Phi]'[t]->action[[2]]}/.solutionFinal;
If[time==0,plotMom=Plot[Evaluate[momentum],{t,time,time+\[Delta]t},PlotStyle-> {Red,Green,Blue},PlotLegends->{"J1","J2","J3"},AxesLabel->{"t","Momentum"}],plotMom=Plot[Evaluate[momentum], {t, time, time+\[Delta]t},PlotStyle-> {Red,Green,Blue}]];
AppendTo[listOfMomentumPlots,plotMom];
(*initialConditions=(({x[time+\[Delta]t]\[Equal]x[t],y[time+\[Delta]t]\[Equal]y[t],\[Theta][time+\[Delta]t]\[Equal] \[Theta][t],x'[time+\[Delta]t]\[Equal]x'[t],y'[time+\[Delta]t]\[Equal]y'[t],\[Theta]'[time+\[Delta]t]\[Equal]\[Theta]'[t],\[Theta]1[time+\[Delta]t]\[Equal]\[Theta]1[t],\[Theta]1'[time+\[Delta]t]\[Equal]\[Theta]1'[t]}/.solutionFinal)/.{t\[Rule]time+\[Delta]t})[[1]]*)initialConditions=(({x[time+\[Delta]t]==x[t],y[time+\[Delta]t]==y[t],\[Theta][time+\[Delta]t]== \[Theta][t],\[Theta]1[time+\[Delta]t]==\[Theta]1[t],\[Theta]1'[time+\[Delta]t]==\[Theta]1'[t]}/.solutionFinal)/.{t->time+\[Delta]t})[[1]]]


(* ::Input::Initialization:: *)
(*Show[listOfMomentumPlots,PlotRange\[Rule] All]*)


(* ::Input::Initialization:: *)
trajectories=Show[listOfPlots,PlotRange-> All];


(* ::Input::Initialization:: *)
Export["trajectories"<>timeAndDateString<>".pdf",trajectories];


(* ::Input::Initialization:: *)
(*Show[listOfMomentumPlots,PlotRange\[Rule] All]*)


(* ::Input::Initialization:: *)
\[Phi]Dots=Last/@listOfActions;


(* ::Input::Initialization:: *)
t\[Phi]dotpairs=Partition[Riffle[Range[Length[#]],#]&@(Last/@listOfActions),2];


(* ::Input::Initialization:: *)
this=t\[Phi]dotpairs/.{thyme_Integer,rate_Real}-> {rate,(t<thyme)};


(* ::Input::Initialization:: *)
\[Phi]dot[t_]:=Piecewise/@{this};


(* ::Input::Initialization:: *)
timelist=Range[lengthOfEpisode];


(* ::Input::Initialization:: *)
\[Phi]solved[t_,tlower_,tupper_]:=NIntegrate[\[Phi]dot[t],{t,tlower,tupper}];


(* ::Input::Initialization:: *)
\[Phi]ForAnimation=ListInterpolation[\[Phi]solved[t,0,#]&/@timelist,{{0,lengthOfEpisode}}];


(* ::Input::Initialization:: *)
\[Phi]plot=Plot[\[Phi]ForAnimation[t],{t,0,10},PlotRange->All];


(* ::Input::Initialization:: *)
Export["phiplot"<>timeAndDateString<>".pdf",\[Phi]plot];


(* ::Input::Initialization:: *)
phidotplot=Plot[Piecewise/@{this},{t,0,lengthOfEpisode},PlotRange->{-1,1}];


(* ::Input::Initialization:: *)
Export["phidotplot"<>timeAndDateString<>".pdf",phidotplot];


(* ::Input::Initialization:: *)
policy=Show[listOfActionPlots,PlotRange-> All];


(* ::Input::Initialization:: *)
Export["policy"<>timeAndDateString<>".pdf",policy];


(* ::Input::Initialization:: *)
movieFrameQ[time_,step_]:=Show[{body,head,tail,tailPivotLine1,tailPivotDot,tailPivotLine2,headPivotDot,headPivotLine1,headPivotLine2}/.robotParameters/.{\[Phi][t]-> \[Phi]ForAnimation[t]}/.listOfSol[[step]]/.{t->time},PlotRange-> {{-(10 b + gap) , 10 b + gap}, {-(10 b + gap ) , 10 b + gap }} /. robotParameters];


(* ::Input::Initialization:: *)
numberOfFrames = 20;


(* ::Input::Initialization:: *)
movieFrames=Table[Table[movieFrameQ[k,j],{k,j-1,j,\[Delta]t/numberOfFrames}],{j,1,lengthOfEpisode}]//Flatten;


(* ::Text::Initialization:: *)
(*The graphics issues thrown, I think, are a product of the objects pointing to states x[t], y[t], \[Theta][t], etc., and \[Phi][t] are different. All states except \[Phi][t] are InterpolatingFunctions while \[Phi][t] is not.*)


(* ::Input::Initialization:: *)
(*ListAnimate[movieFrames]*)


(* ::Input::Initialization:: *)
Export["threelink_vary_k_headspeed_"<>timeAndDateString<>".avi",movieFrames,"FrameRate"-> 20];


(* ::Input::Initialization:: *)
Export["Qmatrix_"<>timeAndDateString<>".dat",Qfinal];
