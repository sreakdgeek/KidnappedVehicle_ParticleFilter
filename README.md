# Particle Filters

Particle Filters are a non-parameteric filters and can operate efficiently in multi-modal continuous sate space.
Using an initial random guess of the particles, Particle filters improve the accuracy by sampling the particles in such a 
way that the most likely particles survive during sampling and contribute to better estimate over time. Particle filters are 
useful in applications such as localizing a vehicle using its noisy GPS measurements of landmark points , 
a posterior probabily of vehicles position in a map is computed using a genetic mutation-selection sampling approach.


[//]: # (Image References) 
[image1]: ./images/ParticleFilters_Schematic.png
[image2]: ./images/ParticleFilters_Algorithm.png
[image2]: ./images/Results.png

Use of Particle filters for Robot motion and localization is described here: 
http://robots.stanford.edu/papers/thrun.pf-in-robotics-uai02.pdf

### Particle Filters - Process

![alt text][image1]

---

### Particle Filters - Algorithm

![alt text][image2]

---

### Results

![alt text][image3]


