//ttvm = trajectories time varying motion

//ECMAScript module

import * as hlao from 'matrix-computations';
import * as ludcmp from 'lu-decomposition';

//To do:
//   - Need more test cases for tpoly()

//'Time and Motion' (quintic polynomial) page 43 from 'Robotics, Vision and Control'.
//s0 - initial (pose - 1D, for example one of [x,y,z,wx,wy,wz]^T)
//sf - final (pose - 1D)
//period - final time (for example 1.0 second)
//samples - number of steps
//v0 - initial velocity
//vf - final velocity
function tpoly(s0,sf,period,samples,v0,vf){
    var debug = 0;
    
    var T = period;
    
    if(typeof(v0) === 'undefined') v0 = 0.0;
    if(typeof(vf) === 'undefined') vf = 0.0;
    
    if(debug) console.log('v0:');
    if(debug) console.log(v0);
    if(debug) console.log('vf:');
    if(debug) console.log(vf);
    
    var S = [[s0],[sf],[v0],[vf],[0.0],[0.0]];
    if(debug) console.log('S vector:');
    if(debug) console.log(S);
    
    var A = [
        [               0.0,                0.0,               0.0,           0.0, 0.0, 1.0],
        [     Math.pow(T,5),      Math.pow(T,4),     Math.pow(T,3), Math.pow(T,2),   T, 1.0],
        [               0.0,                0.0,               0.0,           0.0, 1.0, 0.0],
        [ 5.0*Math.pow(T,4),  4.0*Math.pow(T,3), 3.0*Math.pow(T,2),         2.0*T, 1.0, 0.0],
        [               0.0,                0.0,               0.0,           2.0, 0.0, 0.0],
        [20.0*Math.pow(T,3), 12.0*Math.pow(T,2),             6.0*T,           2.0, 0.0, 0.0]
    ];
    if(debug) console.log('A matrix:');
    if(debug) console.log(A);
    var m = A.length; //number of rows ('A' is a square matrix)
    
    //pad with dummy values
    //   - include an extra zero per row
    for(var i=0;i<A.length;i=i+1){
        A[i].unshift(0.0);
    }
    //   - include an extra row padded with zeros
    var dummyRow = []
    for(var i=0;i<(A[0].length + 1);i=i+1){
        dummyRow.push(0.0);
    }
    A.unshift(dummyRow);
    
    //solve the inverse of 'A'
    var Ainverse = ludcmp.matrixInverseLU(A,m);
    
    //clean-up, remove dummy values
    //   - remove the dummy row
    Ainverse.shift();
    //   - remove the dummy column
    for(var i=0;i<Ainverse.length;i=i+1){
        Ainverse[i].shift();
    }
    if(debug) console.log('A inverse:');
    if(debug) console.log(Ainverse);
    
    //calc. the coefficient vector
    var coeff = hlao.matrix_multiplication(Ainverse,S);
    if(debug) console.log('coefficient vector: ' + coeff);
    
    //create the displacement, velocity, acceleration data
    var S = [];
    var Sd = [];
    var Sdd = [];
    var T = [];
    for(var i=0;i<=samples;i=i+1){
        T[i] = i*(period/samples);
        S[i] = hlao.vector_dot(coeff,[[Math.pow(T[i],5)], [Math.pow(T[i],4)], [Math.pow(T[i],3)], [Math.pow(T[i],2)], [T[i]], [1.0]]); //displacement
        Sd[i] = hlao.vector_dot(coeff,[[5.0*Math.pow(T[i],4)], [4.0*Math.pow(T[i],3)], [3.0*Math.pow(T[i],2)], [2.0*T[i]], [1.0], [0.0]]); //velocity
        Sdd[i] = hlao.vector_dot(coeff,[[20.0*Math.pow(T[i],3)], [12.0*Math.pow(T[i],2)], [6.0*T[i]], [2.0], [0.0], [0.0]]); //acceleration
    }
    
    return {time:T, S:S, Sdot:Sd, Sdotdot:Sdd};
}

//Robotics: Modelling, Planning and Control. Page 167.
//Triangular profile.
function lspb(qi,qf,tf,nsteps){
    //ref: page 167.
    //var tf = 1.0; //final time.
    //var qi = 0.0; //initial position.
    //var qf = Math.PI; //final position.
    //
    //ref: page 269.
    //var tf = 0.5; //final time.
    //var qi = -1.0*Math.PI/2.0; //initial position.
    //var qf = 0.0; //final position.
    //
    //var nsteps = 100;
    var qt; //current position.
    var dt = tf/nsteps; //time step.
    var data = [];
    
    var qcdd = (4.0*Math.abs(qf - qi))/Math.pow(tf,2); //constant acceleration.
    var tc = (tf/2.0) - 0.5*Math.sqrt((Math.pow(tf,2)*qcdd - 4.0*(qf-qi))/qcdd); //time at the end of the parabolic segment.
    
    for(var t=0.0;t<=tf;t=t+dt){
        if((t >= 0) && (t <= tc)){ //                   0 <= t <= tc
            qt = qi + 0.5*qcdd*Math.pow(t,2);
        } else if((t > tc)&&(t <= (tf - tc))){ //     tc  <  t <= tf - tc
            qt = qi + qcdd*tc*(t-tc/2.0);
        } else if((t > (tf - tc))&&(t <= tf)){ // tf - tc <  t <= tf
            qt = qf - 0.5*qcdd*Math.pow((tf-t),2);
        } else {
            console.log("WARNING: t out of bounds.");
        }
        data.push([t,qt]);
    }
    
    return data;
}

/*
//multi-joint trajectory
var qz = [0,0,0,0,0,0];
var qr = [0,Math.PI/2.0,-Math.PI/2.0,0,0,0];
var jm = ttvm.jtraj(qz,qr,1.0,10); //joint motion
console.log(jm);
*/
function jtraj(q0,q1,period,samples,v0,vf){
    var traj = [];
    for(var i=0;i<q0.length;i=i+1){
        traj[i] = tpoly(q0[i],q1[i],period,samples,v0[i],vf[i]);
    }
    return(traj);
}

//Multi-Segment Trajectories (Port from Peter Corke MATLAB mstraj.m (Robotics Toolbox))
//To do:
//   - improve estimate distance travelled during the blend
//   - the for loop for 'q0' calc. needs work
//   - 'slowest' not used
//
//All axes reach their via points at the same time.
//
//Linear motion while polynomial blends connect the segments.
//
//segments - via points (matrix)
//qdmax - speed limits (vector)
//tsegment - durations (vector)
//q0 - initial axis coordinates (vector)
//dt - time step (scalar)
//Tacc - acceleration time (scalar)
//(QD0 - initial axis velocities)
//(QDF - final axis velocities)
//(varargin - show details)
//
//return [TG, taxis]
function mstraj(segments, qdmax, tsegment, q0, dt, Tacc){
    var debug = 0;
    
    //1 row per via point, one column per axis.
    var ns = segments.length;
    var nj = segments[0].length;
    if(debug) console.log("Number of via points: " + ns);
    if(debug) console.log("Number of axes: " + nj);
    
    //Only one of qdmax or tsegment
    //qdmax.length = 0 or tsegment.length = 0
    
    //initialization
    var qd0 = hlao.zeros_vector(nj,'row'); //check for args QD0
    var qdf = hlao.zeros_vector(nj,'row'); //check for args QDF
    var arrive = []; //record planned time of arrival at via points
    var tg = [];
    var taxis = [];
    
    //initial condition
    var q_prev = q0; //(row vector)
    var qd_prev = qd0; //(row vector)
    var clock = 0; //keep track of time
    
    for(var seg=0;seg<ns;seg=seg+1){
    //for(var seg=0;seg<2;seg=seg+1){
        if(debug) console.log("q_prev:");
        if(debug) hlao.print_vector(q_prev);
        if(debug) console.log("qd_prev");
        if(debug) hlao.print_vector(qd_prev);
        if(debug) console.log("clock: " + clock);
        
        //set the blend time, just half an interval for the first segment
        if(Tacc.length > 1) var tacc = Tacc[seg];
        else var tacc = Tacc;
        tacc = Math.ceil(tacc/dt)*dt;
        var tacc2 = Math.ceil(tacc/2.0/dt)*dt;
        if(debug) console.log("tacc: " + tacc);
        if(debug) console.log("tacc2: " + tacc2);
        
        if(seg === 0) var taccx = tacc2;
        else var taccx = tacc;
        if(debug) console.log("taccx: " + taccx);
        
        //estimate travel time
        //    could better estimate distance travelled during the blend
        var q_next = []; //current target (row vector)
        for(var i=0;i<nj;i=i+1){
            q_next.push(segments[seg][i]);
        }
        var dq = hlao.vector_arithmetic(q_next,q_prev,'-'); //total distance to move this segment
        if(debug) console.log("q_next:");
        if(debug) hlao.print_vector(q_next);
        if(debug) console.log("dq");
        if(debug) hlao.print_vector(dq);
        
        var tseg; var slowest;
        if(qdmax.length > 0){
            //qdmax is specified, compute slowest axis
            
            //distance moved during blend
            var qb = hlao.vector_multiplication_scalar(qdmax,taccx/2.0); //IMPORTANT: is this being used?
            var tb = taccx;
            if(debug) console.log("qb");
            if(debug) hlao.print_vector(qb);
            if(debug) console.log("tb: " + tb);
            
            //convert to time
            var dq_abs = [];
            for(var i=0;i<dq.length;i=i+1){
                dq_abs.push(Math.abs(dq[i]));
            }
            var tl = [];
            for(var i=0;i<dq.length;i=i+1){
                tl[i] = dq_abs[i]/qdmax[i];
                tl[i] = Math.ceil(tl[i]/dt)*dt;
            }
            if(debug) console.log("dq_abs");
            if(debug) hlao.print_vector(dq_abs);
            if(debug) console.log("tl");
            if(debug) hlao.print_vector(tl);
            
            //find the total time and slowest axis
            var tbv = hlao.ones_vector(tl.length,'row');
            tbv = hlao.vector_multiplication_scalar(tbv,tb);
            var tt = hlao.vector_arithmetic(tbv,tl,'+');
            taxis.push(tt);
            tseg = Number.MIN_VALUE;
            for(var i=0;i<tt.length;i=i+1){
                if(tseg < tt[i]) tseg = tt[i];
            }
            if(debug) console.log("tbv");
            if(debug) hlao.print_vector(tbv);
            if(debug) console.log("tt");
            if(debug) hlao.print_vector(tt);
            
            //best if there is some linear motion component
            if(tseg <= 2.0*tacc) tseg = 2.0*tacc;
            if(debug) console.log("tseg: " + tseg);
        } else if(tsegment.length > 0){
            //segment time specified, use that
            tseg = tsegment[seg];
            slowest = NaN;
        }
        
        //log the planned arrival time
        arrive[seg] = clock + tseg;
        if(seg > 1) arrive[seg] = arrive[seg] + tacc2;
        if(debug) console.log("arrive: " + arrive);
        
        //create the trajectories for this segment
        
        //linear velocity from qprev to qnext
        var qd = hlao.vector_multiplication_scalar(dq,1.0/tseg);
        if(debug) console.log("qd");
        if(debug) hlao.print_vector(qd);
        
        //add the blend polynomial
        var qf = hlao.vector_arithmetic(q_prev,hlao.vector_multiplication_scalar(qd,tacc2),'+');
        if(debug) console.log("qf");
        if(debug) hlao.print_vector(qf);
        var period = taccx; //final time (Hence T is from 0 to 49)
        var samples = (taccx/dt); //number of steps
        if(debug) console.log("q0 (q):");
        if(debug) hlao.print_vector(q0);
        if(debug) console.log("qf (q_prev+tacc2*qd):");
        if(debug) hlao.print_vector(qf);
        if(debug) console.log("period: " + period);
        if(debug) console.log("samples: " + samples);
        if(debug) console.log("qd_prev:");
        if(debug) hlao.print_vector(qd_prev);
        if(debug) console.log("qd:");
        if(debug) hlao.print_vector(qd);
        //qb = jtraj(q, q_prev+tacc2*qd, 0:dt:taccx, qd_prev, qd);
        var qb = jtraj(q0,qf,period,samples,qd_prev,qd);
        if(debug) console.log("qb length: " + qb.length);
        if(debug) console.log("qb");
        if(debug) console.log(qb);
        if(debug) console.log('qb[0].S.length (i): ' + qb[0].S.length);
        if(debug) console.log('qb.length (j):');
        if(debug) console.log(qb.length);
        for(var i=1;i<qb[0].S.length;i=i+1){ //for each axis data point (drop first data point)
            var qh = [];
            for(var j=0;j<qb.length;j=j+1){ //for each axes
                //if(!isNaN(qb[i].S[j])) qh.push(qb[i].S[j]);
                if(debug) console.log('j, i:');
                if(debug) console.log(j + ', ' + i);
                if(debug) console.log('qb[j].S[i]:');
                if(debug) console.log(qb[j].S[i]);
                if(!isNaN(qb[j].S[i])) qh.push(qb[j].S[i]);
            }
            if(qh.length > 0) tg.push(qh);
        }
        if(debug) console.log("tg length: " + tg.length);
        if(debug) console.log("tg");
        if(debug) console.log(tg);
        
        clock = clock + taccx; //update the clock
        
        //add the linear part, from tacc/2+dt to tseg-tacc/2
        var cnt = 0;
        if(debug) console.log('tseg = ' + tseg);
        if(debug) console.log('tacc2 = ' + tacc2);
        if(debug) console.log('tseg-tacc2 = ' + tseg-tacc2);
        for(var t=(tacc2+dt);t<(tseg-tacc2+dt/2.0);t=t+dt){ //IMPORTANT: check this.
            if(debug) console.log("t: " + t);
            var s = t/tseg;
            //console.log("s: " + s);
            var q0 = hlao.vector_arithmetic(hlao.vector_multiplication_scalar(q_prev,(1.0-s)),hlao.vector_multiplication_scalar(q_next,s),'+'); //linear step (IMPORTANT: 'q0' redefined)
            //console.log("q0 (q)");
            //hlao.print_vector(q0);
            tg.push(q0);
            clock = clock + dt;
            cnt = cnt + 1;
        }
        if(debug) console.log("cnt: " + cnt);
        if(debug) console.log("tg");
        if(debug) console.log(tg);
        if(debug) console.log("tg length: " + tg.length);
        
        q_prev = q_next; //next target becomes previous target
        qd_prev = qd;
    }
    
    //add the final blend
    var period = tacc2; //final time (Hence T is from 0 to 49)
    var samples = (tacc2/dt); //number of steps
    var qb = jtraj(q0,q_next,period,samples,qd_prev,qdf); //IMPORTANT: need to fix.
    if(debug) console.log("qb");
    if(debug) console.log(qb);
    for(var i=1;i<qb[0].S.length;i=i+1){ //for each axis data point (drop first data point)
        var qh = [];
        for(var j=0;j<qb.length;j=j+1){ //for each axes
            //if(!isNaN(qb[i].S[j])) qh.push(qb[i].S[j]);
            if(!isNaN(qb[j].S[i])) qh.push(qb[j].S[i]);
        }
        if(qh.length > 0) tg.push(qh);
    }
    if(debug) console.log("tg");
    if(debug) console.log(tg);
    if(debug) console.log("tg length: " + tg.length);
    
    //var t = (0:numrows(tg)-1)'*dt;
    //check if 'taxis' is the same as 't'
    var t = [];
    for(var i=0;i<tg.length;i=i+1){
        t.push(i*dt);
    }
    
    return([tg,t]);
}

//gradient - numerical derivative. ref: Numerical recipes in C, page 187.
//IMPORTANT: ill defined at k == 0 and k == f.length - 1
//Solution: pad with zero at k == 0 if position x0 is 0 etc.
//f[
//  [x0,y0],
//  [x1,y1],
//  ...,
//  [xn,yn]
//];
function gradient(f){
    var df;
    var data = [];
    for(var k=0;k<f.length;k=k+1){
        if(k == 0) df = (f[k+1][1] - f[k][1])/(f[k+1][0] - f[k][0]);                   // k = 0
        else if(k == (f.length - 1)) df = (f[k][1] - f[k-1][1])/(f[k][0] - f[k-1][0]); // k = f.length - 1
        else df = (f[k+1][1] - f[k-1][1])/(f[k+1][0] - f[k-1][0]);                     // 0 < k < f.length - 1
        data.push([f[k][0],df]);
    }
    return data;
}

export {
    tpoly,
    lspb,
    jtraj,
    gradient,
    mstraj
};