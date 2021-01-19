# trajectories
Trajectories - time varying motion

## Dependencies

There are 2 dependencies 'matrix-computations' and 'lu-decomposition'.

```bash
https://github.com/PeterTadich/matrix-computations
https://github.com/PeterTadich/lu-decomposition
```

## Installation

### Node.js

```bash
npm install https://github.com/PeterTadich/trajectories
```

### Google Chrome Web browser

No installation required for the Google Chrome Web browser.

## How to use

### Node.js

```js
import * as ttvm from 'trajectories';
```

### Google Chrome Web browser

```js
import * as ttvm from './ttvm.mjs';
```

## Examples

### Node.js (server side)

Copy the following code to index.mjs

```js
/*
%MATLAB
[s,sd,sdd] = tpoly(0, 1, 50); %page 44 Robotics, Vision and Control
*/
import * as ttvm from 'trajectories';

var period = 50.0-1.0; //final time (Hence T is from 0 to 49)
var samples = 50-1; //number of steps
var traj = ttvm.tpoly(0.0,1.0,period,samples); //generate quintic polynomial trajectory
var T = traj.time; var s = traj.S; var sd = traj.Sdot; var sdd = traj.Sdotdot;
```

Then run:

```bash
npm init -y
npm install https://github.com/PeterTadich/trajectories
node index.mjs
```

If the above does not work, modify the package.json file as follows:
Helpful ref: [https://stackoverflow.com/questions/45854169/how-can-i-use-an-es6-import-in-node-js](https://stackoverflow.com/questions/45854169/how-can-i-use-an-es6-import-in-node-js)

```js
"scripts": {
    "test": "echo \"Error: no test specified\" && exit 1",
    "start": "node --experimental-modules index.mjs"
  },
"type": "module",
```

Now try:

```bash
npm start
```

## Examples (extended)

Quintic polynomial trajectory - tpoly().

```js
/*
%MATLAB
t = 0.0:0.02:1.0; % 0.0 to 1.0 (51 samples)
[s,sd,sdd] = tpoly(0, 1, t);
*/
import * as ttvm from 'trajectories';

var period = 1.0; //final time (Hence T is from 0.0 to 1.0)
var samples = 50.0; //number of steps
var traj = ttvm.tpoly(0.0,1.0,period,samples);
var T = traj.time; var s = traj.S; var sd = traj.Sdot; var sdd = traj.Sdotdot;
console.log("'T' - time:");
console.log(T);
console.log("'s' - linear 1D pose:");
console.log(s);
console.log("'sd' - first derivative:");
console.log(sd);
console.log("'sdd' - second derivative:");
console.log(sdd);
```

Quintic polynomial trajectory - tpoly().

```js
/*
%MATLAB
t = 0.0:0.02:1.0; % 0.0 to 1.0 (51 samples)
[s,sd,sdd] = tpoly(0, 1, t);
*/
import * as ttvm from 'trajectories';

var period = 1.0; //final time (Hence T is from 0.0 to 1.0)
var samples = 50.0; //number of steps
var traj = ttvm.tpoly(0.0,1.0,period,samples);
var T = traj.time; var s = traj.S; var sd = traj.Sdot; var sdd = traj.Sdotdot;
console.log("'T' - time:");
console.log(T);
console.log("'s' - linear 1D pose:");
console.log(s);
console.log("'sd' - first derivative:");
console.log(sd);
console.log("'sdd' - second derivative:");
console.log(sdd);
```

Single/Multi-dimensional axis of motion - jtraj().

```js
/*
%MATLAB
[s,sd,sdd] =  mtraj(@tpoly, [0 2], [1 -1], 50); %page 46 Robotics, Vision and Control
*/
import * as ttvm from 'trajectories';

var q0 = [0.0,2.0];
var qf = [1.0,-1.0];
var period = 50.0-1.0; //final time (Hence T is from 0 to 49)
var samples = 50-1; //number of steps
var jm = ttvm.jtraj(q0,qf,period,samples); //joint motion
console.log(jm);
```

Single/Multi-dimensional axis of motion - jtraj().

```js
/*
%MATLAB
[s,sd,sdd] =  mtraj(qz,qr,10); %page 192 Robotics, Vision and Control
*/
import * as ttvm from 'trajectories';

var qz = [0,0,0,0,0,0];
var qr = [0,Math.PI/2.0,-Math.PI/2.0,0,0,0];
var period = 10.0-1.0; //final time (Hence T is from 0 to 49)
var samples = 10-1; //number of steps
var jm = ttvm.jtraj(qz,qr,period,samples); //joint motion
console.log(jm);
```

Multi-segment multi-axis trajectory of motion - mstraj().

```js
/*
%MATLAB
%page 47 Robotics, Vision and Control
via = [ 4,1; 4,4; 5,2; 2,5 ];
%q = mstraj(via, [2,1], [], [4,1], 0.05, 0.0); % Tacc = 0.0
q = mstraj(via, [2,1], [], [4,1], 0.05, 1.0); % Tacc = 1.0
*/
import * as ttvm from 'trajectories';
import * as hlao from './node_modules/matrix-computations';

var via = [[4.0,1.0],[4.0,4.0],[5.0,2.0],[2.0,5.0]];
var qdmax = [2.0,1.0];
var tsegment = [];
var q0 = [4.0,1.0];
var dt = 0.05;
var Tacc = 0.0;
//var Tacc = 1.0;
var tg; var t;
[tg,t] = ttvm.mstraj(via, qdmax, tsegment, q0, dt, Tacc);
hlao.print_matrix(tg);
console.log('tg length: ' + tg.length);
```

Multi-segment multi-axis trajectory of motion - mstraj().

```js
/*
%MATLAB
%page 162 Robotics, Vision and Control
path = [ 1 0 1; 1 0 0; 0 0 0; 0 2 0; 1 2 0;1 2 1; 0 1 1; 0 1 0; 1 1 0; 1 1 1];
p = mstraj(path, [0.5 0.5 0.3], [], [2 2 2], 0.02, 0.2);
*/
import * as ttvm from 'trajectories';
import * as hlao from './node_modules/matrix-computations';

var path = [[1,0,1],[1,0,0],[0,0,0],[0,2,0],[1,2,0],[1,2,1],[0,1,1],[0,1,0],[1,1,0],[1,1,1]];
var qdmax = [0.5,0.5,0.3];
var tsegment = [];
var q0 = [2,2,2];
var dt = 0.02;
var Tacc = 0.2;
var tg; var t;
[tg,t] = ttvm.mstraj(path, qdmax, tsegment, q0, dt, Tacc);
hlao.print_matrix(tg);
console.log('tg length: ' + tg.length);
```

Multi-segment multi-axis trajectory of motion - mstraj().

```js
/*
%MATLAB
%page 165 Robotics, Vision and Control
xf = 50; xb = -xf; y = 50; zu = 20; zd = 50;
path = [xf y zd; xb y zd; xb y zu; xf y zu; xf y zd] * 1e-3;
p = mstraj(path, [], [0, 3, 0.25, 0.5, 0.25]', path(1,:), 0.01, 0);
*/
import * as ttvm from 'trajectories';
import * as hlao from './node_modules/matrix-computations';

var xf = 50 * 1e-3; var xb = -xf; var y = 50 * 1e-3; var zu = 20 * 1e-3; var zd = 50 * 1e-3;
var path = [[xf,y,zd],[xb,y,zd],[xb,y,zu],[xf,y,zu],[xf,y,zd]];
var qdmax = [];
var tsegment = [0, 3, 0.25, 0.5, 0.25];
var q0 = path[0];
var dt = 0.01;
var Tacc = 0.0;
var tg; var t;
[tg,t] = ttvm.mstraj(path, qdmax, tsegment, q0, dt, Tacc);
hlao.print_matrix(tg);
console.log('tg length: ' + tg.length);
```

## License

[MIT](LICENSE)