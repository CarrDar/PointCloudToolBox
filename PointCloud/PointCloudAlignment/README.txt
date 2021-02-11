------------------------------------------------------------------------------
Projet : Alignment of point clouds
Author : Neal Gauvin

------------------------------------------------------------------------------
Description :
Matlab Toolkit for the alignment of point clouds. It allows to test and compare different Iterative Closest Point (ICP) methods, to measure the errors and how they propagate. This in a transparent way.

In particular, it contains :
 - ICPVarOut : a class to efficiently store the interesting variables computed in each steps of an iterative algorithm, like for instance the CPU, the status of the iteration, transform matrices, distances and errors as well :
       (1),(2) Mean distance between measured matched positions (mean and rms).
       (3) Mean distance between true matched positions.
       (4) Mean distance between true positions irrespectively of any matching.
          In a perfect world this should be close to 0, therefore giving an
          idea of the "error" in the alignment.
       (5)=(4)-(3) error of the matching.

 - DemoSim.m and DemoReal : demo scripts to start aligning point clouds.

------------------------------------------------------------------------------
More about the alignment...

The ICP concept has been cut in 7 different stages :

 - initial alignmement.

 - Selection of some points in one or both meshes. 
   Use all available points.
   Uniform subsampling [Turk 94].
   Random sampling (different in each iteration [Masuda 96])
   Choose points such that the distribution of normals/curvatures among selected points is as large as possible.
   Color/intensity sampling.
   Smoothing of point cloud by averaging boxes of given size.
   Smoothing of point cloud by using splines.
   Future :

The following steps are implemented in the ICP.m function.
This function receives two PointClouds classes : it will attempt to align the source point cloud on the target one.
It welcomes also a ICPVarOut class to store some interesting variables in each iteration (see above).
And numerous parameters !

 - Matching the selected points to the points in the target.
   Implemented :
   Find the closest point. 
   This can be accelerated with the help of a k-d tree, a GL tree or a Delaunay triangulation.
   Other that may be implemented :
   closest-point caching [Simon 96](the quickest ?))
   Closest compatible point (normals within 45°).
   Find the intersection of the ray originating at the source point in the direction of the source point's normal with the target surface (normal shooting) [Chen 91].
   Normal shooting (see above) to compatible points (normals within 45°).
   Project the source point onto the desitnation mesh's range camera ("reverse calibration") [Blais 95, Neugebauer 97].
   Project the source point onto the target mesh, the perform a search in the target range image. 
   The search may use a metric based point-to-point distance [Benjemaa 97], point-to-ray distance [Dorai 98], or compatibility of intensity [Weik 97] or color [Pulli 97].

 - Weighting the corresponding pair appropriately.
   Implemented :
   Constant weight.
   Weights based on point-to-point distances (~ < threshold) [Rus 01, Godin 94].
   Weighting based on compatibility of normals/curvatures [Rus 01].
   Other that may be implemented :
   Weighting based on the expected effect of scanner noise on the uncertainty in the error metric [Rus 01].

 - Rejecting certain pairs based on looking at each pairs appropriately.
   Implemented :
   Difference > X° in normals.
   With large point-to point distances.
   Reject n% of the more distance pairs.
   Reject pairs with edges, 
   Rejection of pairs whose distance  is larger to some multiple of the standard deviation [Masuda 96].
   Rejection of pairs not consistent with neighbouring pairs (assuming surface move rigidly) [Dorai 98].

 - Assignment of an error metric based on the point pairs. 
   Implemented :
   Least squared distances between corresponding points. 
   (2 possibilites : the mean and the standard deviation.)
   Many closed-form solutions, differences small [Eggert 97]. 
   In physical terms this metric can be visualized as the total potential energy of springs attached between matched point pairs.
   Sum of squared distances from each source point to the plane containing the destination point and oriented perpendicular to the destination normal [Chen 91]. No closed-form solutions are available. 
   Two solutions : 
       a generic non linear method (e.g. least squares Levenberg-Marquardt algorithm ) [Fitz 03], or 
       linearisation (i.e., assuming incremental rotations are small, so sint=t and cost=1) [Low 04].
   The physical interpretation might be the total potential energy of springs that are free to move on model point tangent planes and fixed to the data points.
   Note that the ICP.m file also provide some nonlinear non-ICP methods, which allows to deal with resizing and shearing. See the doc for details.
   Future :
   Pseudo plane to plane [Low 03]. 

 - Minimising the error pair.
   There are several ways to formulate the search for the alignment:
    - Repeatedly generating a set of corresponding points using the current transformation, and finding a new transformation that minimizes the error metric [Chen 91].
    - The above iterative minimization, combined with extrapolation in transform space to accelerate convergence [Besl 92].
    - Performing the iterative minimisation starting with several perturbations in the initial conditions, then selecting the best result [Simon 96]. This avoids spurious local minima in the error function, especially when the point-to-point error metric is used.
   Other way not implemented :
    - Performing the iterative minimisation using various randomly-selected subsets of points, then selecting the optimal result using a robust (least median of squares) metric [Masuda 96] (not implemented).
    - Stochastic search for the best transform, using simulated annealing [Blais 95] (not implemented).

An example how to perform multi-resolution smoothing to help the ICP algorithm to converge is given in MultiResoICP.

------------------------------------------------------------------------------



------------------------------------------------------------------------------
References :

[Benjemaa 97] Benjemaa, R. and Schmitt, F. "Fast Global Registration of 3D Sampled Surfaces Using a Multi-Z-Buffer Technique", Proc. 3DIM, 1997.
[Besl 92]   Besl, P. and McKay, N. "A Method for Registration of 3-D Shapes", Trans. PAMI, Vol. 14, No. 2, 1992.
[Blais 95]  Blais, G. and Levine, M. "Registering Multiview Range Data to Create 3D Computer Objects", Trans. PAMI, Vol. 17, No. 8, 1995.
[Chen 91]   Chen, Y. and Medioni, G. "Object Modeling by Registration of Multiple Range Images", Proc. IEEE Conf. on Robotics and Automation, 1991.
[Dorai 98]  Dorai, C.,Weng, J., and Jain, A. "Registration and Integration of Multiple Object Views for 3D Model Constrution", Trans. PAMI, Vol. 20, No. 1, 1998.
[Eggert 97] Eggert, D. W., Lorusso, A., and Fisher, R. B. "Estimating 3-D Rigid Body Transformations: A Comparison of Four Major Algorithms," MVA, Vol. 9, No. 5/6, 1997.
[Fitz 03]   Andrew W. Fitzgibbon. "Robust registration of 2d and 3d point sets". Image and Vision Computing, 21(13-14):1145{1153, 2003.
[Godin 94]  Godin, G., Rioux, M., and Baribeau, R. "Three-dimensional Registration Using Range and Intensity Information," Proc. SPIE: Videometrics III, Vol. 2350, 1994.
[Low 03]    K.-L. Low, A. Lastra, "Reliable and rapidly-converging ICP Algorithm using Multiresolution Smoothing."
[Low 04]    K.-L. Low, "Linear Least-Squares Optimisation for Point-to-Plane ICP Surface registration.
[Masuda 96] Masuda, T., Sakaue, K., and Yokoya, N. "Registration and Integration of Multiple Range Images for 3-D Model Construction", Proc. CVPR, 1996.
[Neugebauer 97] Neugebauer, P. "Geometrical Cloning of 3D Objects via Simultaneous Registration of Multiple Range Images", Proc. SMA, 1997.
[Paul 10] Rasmus R. Paulsen, Jakob A. Baerentzen, and Rasmus Larsen. "Markov random field surface reconstruction". IEEE Transactions on Visualization
and Computer Graphics, 16, 2010.
[Pulli 97] Pulli, K. Surface Reconstruction and Display from Range and Color Data, Ph. D. Dissertation, University of Washington, 1997.
[Rus 01]    S. Rusinkiewicz, M. Leroy, "Effective Variants of the ICP Algorithm". 
[Simon 96]  Fast and Accurate Shape-Based Registration, Ph. D. Dissertation, Carnegie Mellon University, 1996.
[Turk 94]   Turk, G. and Levoy, M. "Zippered Polygon Meshes from Range Images", Proc. SIGGRAPH, 1994.
[Weik 97]   Weik, S. "Registration of 3-D Partial Surface Models Using Luminance and Depth Information", Proc. 3DIM, 1997.

