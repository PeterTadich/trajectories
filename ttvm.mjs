//ttvm = trajectories time varying motion

//ECMAScript module

//'Time and Motion' (quintic polynomial) page 43 from 'Robotics, Vision and Control'.
function tpoly(s0,sf,period,samples){
    var T = period;
    
    var S = [[s0],[sf],[0.0],[0.0],[0.0],[0.0]];

    var A = [
        [               0.0,                0.0,               0.0,           0.0, 0.0, 1.0],
        [     Math.pow(T,5),      Math.pow(T,4),     Math.pow(T,3), Math.pow(T,2),   T, 1.0],
        [               0.0,                0.0,               0.0,           0.0, 1.0, 0.0],
        [ 5.0*Math.pow(T,4),  4.0*Math.pow(T,3), 3.0*Math.pow(T,2),         2.0*T, 1.0, 0.0],
        [               0.0,                0.0,               0.0,           2.0, 0.0, 0.0],
        [20.0*Math.pow(T,3), 12.0*Math.pow(T,2),             6.0*T,           2.0, 0.0, 0.0]
    ];
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
    var Ainverse = matrixInverseLU(A,m);
    
    //clean-up, remove dummy values
    //   - remove the dummy row
    Ainverse.shift();
    //   - remove the dummy column
    for(var i=0;i<Ainverse.length;i=i+1){
        Ainverse[i].shift();
    }
    
    //calc. the coefficient vector
    var coeff = matrix_multiplication(Ainverse,S);
    //console.log('coefficient vector: ' + coeff);
    
    //create the displacement, velocity, acceleration data
    var S = [];
    var Sd = [];
    var Sdd = [];
    var T = [];
    for(var i=0;i<=samples;i=i+1){
        T[i] = i*(period/samples);
        S[i] = vector_dot(coeff,[[Math.pow(T[i],5)], [Math.pow(T[i],4)], [Math.pow(T[i],3)], [Math.pow(T[i],2)], [T[i]], [1.0]]); //displacement
        Sd[i] = vector_dot(coeff,[[5.0*Math.pow(T[i],4)], [4.0*Math.pow(T[i],3)], [3.0*Math.pow(T[i],2)], [2.0*T[i]], [1.0], [0.0]]); //velocity
        Sdd[i] = vector_dot(coeff,[[20.0*Math.pow(T[i],3)], [12.0*Math.pow(T[i],2)], [6.0*T[i]], [2.0], [0.0], [0.0]]); //acceleration
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

export {
    tpoly,
    lspb
};