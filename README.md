# Carla-Controllers
Controls Course Project: Implementing PID (Stanley Control for Lateral Control) and Model Predictive Controller in Carla Simulator. The backbon code interfacing with the Carla was used from the Coursera: Introduction to Self-Driving Cars Course.

<p align="center">
  <img width="640" height="480" src="https://github.com/sapan-ostic/Carla-Controllers/blob/main/results/mpc_carla.gif">
</p>

# Coursera Evaluator scores:
Model Predictive Controller             |  PID Controller
:-------------------------:|:-------------------------:
|100% Waypoints tracked | 87.25% waypoints tracked|
|![](https://github.com/sapan-ostic/Carla-Controllers/blob/main/results/MPC_with_ref_traj.png)  |  ![](https://github.com/sapan-ostic/Carla-Controllers/blob/main/results/pid_speed_profile.png)|
|![](https://github.com/sapan-ostic/Carla-Controllers/blob/main/results/PID_with_ref_traj.png)  |  ![](https://github.com/sapan-ostic/Carla-Controllers/blob/main/results/mpc_speed_profile.png)|



# Installation 
1. Install Carla: [Download here](https://d18ky98rnyall9.cloudfront.net/IFfK-Ce8Eem3Cw5hhdQCGg_210f0c4027bc11e9ae95c9d2c8ddb796_CARLA-Setup-Guide-_Ubuntu_.pdf?Expires=1607904000&Signature=Luzyz99-6Uzg5x3NvJff3HpUQCsH7iOWKiiVjq1EVRFSZc8UY-SrAP8sbxDg0MxOAIDMob1cZNNQL3FfgwkYkR8WwUHEgdGU-FIua5jGs3EbnxPJgVKeYBEQaArKGKh56pspUiOSWcU1tFG~m6crK5aS0YrOCVH1eALf5OT~2M8_&Key-Pair-Id=APKAJLTNE6QMUY6HBC5A) 

2. Download Course1FinalProject.zip into CarlaSimulator folder [Download here](https://d18ky98rnyall9.cloudfront.net/ZQFoJyNEEem3Cw5hhdQCGg_65256a90234411e982bdbb99b90531e3_Course1FinalProject.zip?Expires=1607817600&Signature=VJrunHtmRKcAqHIU8DVvDnZQyHWWSXrr3yUSpZ~H~N~dgEhzu6khmHO-MPOlKmYz45xCE5qzRBekhK16f7-Xe2bgK-YyqpjP-8pm29Kt8NETgW3TOZl7RZDiEqCoMSvWILUdlpdXD-agkDBuIgvX9Df2Gjmyc8MHhpTiCasKJbY_&Key-Pair-Id=APKAJLTNE6QMUY6HBC5A).

3. Install casadi: `pip install casadi`

4. Install Ipopt:
```
wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.4.tgz
tar xvzf Ipopt-3.12.4.tgz

cd Ipopt-3.12.4/ThirdParty/Blas
./get.Blas
cd ../Lapack
./get.Lapack
cd ../Mumps
./get.Mumps
cd ../Metis
./get.Metis
cd ../../                        #Go back to the IPOPT base dir
```
5. Download HSL: [Get Lisensce from here](http://www.hsl.rl.ac.uk/ipopt/)

6. Unpack HSL in Ipopt
```
cd ~/Ipopt-3.12.4/ThirdParty/HSL
tar xvzf coinhsl-2014.01.10.tar.gz
mv coinhsl-2014.01.10 coinhsl
```
7. Compile HSL
```
cd coinhsl-2014.01.10
./configure LIBS="-llapack" --with-blas="-L/usr/lib -lblas" CXXFLAGS="-g -O2 -fopenmp" FCFLAGS="-g -O2 -fopenmp" CFLAGS="-g -O2 -fopenmp"
make -j 4
make install
ln -s /usr/local/lib/libcoinhsl.so /usr/local/lib/libhsl.so
echo  'export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"' >> ~/.bashrc 
```

8. Compile Ipopt:
```
cd ~/Ipopt-3.12.4/
mkdir build
cd build
../configure
make -j 4                        #Compile using 4 cores (if you have them)
make install
```

# Run Instructions 
```
cd CarlaSimulator
./CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=30
```

In new terminal run,
```
cd CarlaSimulator/PythonClient/Course1FinalProject
python3 module_7.py -c MPC
```
Run `python3 module_7.py -h` for more options. 
