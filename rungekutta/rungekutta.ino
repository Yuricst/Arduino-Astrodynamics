// setup
const int n = 6;  // to modify!
double k1[n], k2[n], k3[n], k4[n], utmp[n], state1[n]; 

// ODE parameters
double p[1] = {1.0, };
double steps = 0;
double print_frequency = 20;

// initial condition
double t = 0;
double state0[n] = {1, 0, 0, 0, 1, 0};
double dt = 0.005;
double tf = 0.863;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Beginning integration...");
}

void loop() {
  // put your main code here, to run repeatedly:

  while (t < tf) {
    // correct step-size
    if (tf-t < dt) {
      dt = tf - t;
    }
    // apply Runge-Kutta updates
    RungeKuttaUpdate(t, dt, state0, state1);
    t = t+dt;
    steps++;
    for (int i=0; i<n; i++) {
      state0[i] = state1[i];
    }
    // print statements
    if (steps>=print_frequency) {
      Serial.print("state at t: ");
      Serial.print(t);
      Serial.print("  ");
      for (int i=0; i<n; i++) {
          Serial.print(state1[i], HEX);
          Serial.print("  ");
      }
      Serial.println();
      // reset
      steps = 0;

    }

    // check if done
    if (abs(t-tf) < dt/1000) {
      Serial.println("Done! Result state: ");
      for (int i=0; i<n; i++) {
          Serial.print(state1[i], HEX);
          Serial.print("  ");
      }
      Serial.println();
    }
  }
  
}


void RungeKuttaUpdate(double t, double dt, double u0[], double u1[]) {
  // evaluate k1
  ODE(t, u0, k1);

  // evaluate k2
  for (int i=0; i<n; i++) {
    utmp[i] = u0[i] + k1[i]*dt*0.5;
  }
  ODE(t + dt*0.5, utmp, k2);

  // evaluate k3
  for (int i=0; i<n; i++) {
    utmp[i] = u0[i] + k2[i]*dt*0.5;
  }
  ODE(t + dt*0.5, utmp, k3);

  // evaluate k4
  for (int i=0; i<n; i++) {
    utmp[i] = u0[i] + k3[i]*dt;
  }
  ODE(t + dt, utmp, k4);
  
  // step-update
  for (int i=0; i<n; i++) {
    u1[i] = u0[i] + dt * (k1[i] + 2.0*(k2[i] + k3[i]) + k4[i]) / 6;
  }
}


void ODE(double t, double u[], double du[]) {
  // two-body eom
  double r = sqrt(pow(u[0],2) + pow(u[1],2) + pow(u[2],2));
  // position derivatives
  du[0] = u[3];
  du[1] = u[4];
  du[2] = u[5];
  // velocity derivatives
  du[3] = -p[0]/pow(r,3) * u[0];
  du[4] = -p[0]/pow(r,3) * u[1];
  du[5] = -p[0]/pow(r,3) * u[2];
}
