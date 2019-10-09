# Interactive-Architectural-Modeling


## 1. Introduction
This project is an implementation of the paper “Interactive Architectural Modeling with Procedural Extrusions” [1]. User can use this tool to construct 3D building models from 2D inputs, i.e. polygons and poly lines.
## 2. UI 
The UI consists of 3 Views: Plan Edit, Profile Edit and Preview. The user draws the ground plan in plan edit and profiles associated to the plan in the profile edit. The system automatically generates the corresponding 3D model. An example is shown in Fig. 1.


Figure 1 The UI. From left to right are: Plan Edit, Profile Edit and Preview
### 2.1 Plan Edit
In the Plan Edit (the left most in Fig. 1.), the user draws a set of edges using mouse. These edges must form a valid plan, which consists of a set of non-self intersecting polygons. 

To draw an edge, click mouse once to draw the first point and second time to draw the second point. These two points define the edge. 
To check the input, press [s] key. 
To delete a point, move the mouse to the point and press [Delete] key.
To clear the plan, press [Shift] + [Delete]

NOTE: When using these keys, be aware of which window is the current active window. For example, if you click on the preview window/view (the right most one) and then press [Shift] + [Delete], nothing will be affected in the Plan Edit window/view.
### 2.2 Profile Edit
The Profile Edit is similar to Plan Edit, except that the user is expected to draw the vertical profile. A valid profile should be monotonic in the vertical direction.

The keys are the same as Plan Edit, mentioned in section 2.1.
### 2.3 Preview
The model is automatic generated in this window. If the model disappears, that means the current input is not valid or the algorithm can’t handle it. 

To rotate the model, click and drag the mouse to the corresponding direction
To zoom in and out, press [Up] and [Down] key respectively
## 3. Back-end
This project is developed on Mac OS X 10.8.1, using Qt Creator. Two libraries, i.e. OpenGL and CGAL, are used in the code.

There are 7 CPP files: edge.cpp, edit.cpp, extrusion.cpp, main.cpp, plandata.cpp, point.cpp, render.cpp. Among these, edge.cpp and point.cpp are trivia data structures that are used to store the user inputs. The other files/classes will be discussed in the following sections.
### 3.1 Main.cpp and render.cpp file
The main.cpp and render.cpp implements the UI of the system. The main.cpp implements the Plan Edit window and Profile Edit window. The render.cpp implements the Preview window.

The main.cpp reads in user input and passes the data to class Editor, which is used to store the user data. Then the class Extrusion is used to do the actual generation of the 3D model. Finally, the resulting 3D data is passed to render.cpp to show the model.
### 3.2 class Editor
This class is used to stores and processes the input data.  These input data is stored as a set of edges. When the class Extrusion needs the plan and profile data, the class Editor generates the polygons and polylines from the set of edges.

The class Editor also checks whether the validity of the input.
### 3.3 class PlanData
The class PlanData stores the cross-section data when doing the sweep plane algorithm, which will be discussed shortly. PlanData is a doubly linked list of corners. Each corner has a pointer to the next corner and the previous corner (assuming counter-clockwise order) and a pointer to its previous and next edges, Fig. 3. At the beginning of the algorithm the plan data structure encodes the input plan. During the sweep the data structure is updated to encode any changes to the active plan.

Figure 3 PlanData

### 3.4 class Extrusion
This is the most important class. It extrudes the input plan using a sweep plane algorithm. At each height of the sweep plane a 2D cross-section through the building is another 2D plan. This movement and the implicitly defined geometry are straightforward until an event occurs. During events, it processes modifications to the active plan.
The most important events are Generalized intersection event and edge direction event. Generalized intersection event is processed using the construction and handling of chains, which are composed of the involved edges in the intersection event. Edge direction event happens when the profile changes the direction. 
The events are stored in a priority queue and get pulled out of the queue in the order of heights, hence simulating the sweep process. When the events are processed, the geometry is modified accordingly and new events might be inserted into the priority queue. When all events are processed, the algorithm completes, and the geometry of the 3D model is defined.
## 4. Conclusion
This paper is hard to implement because the handling of the Generalized Intersection Event is not perfect. There are lot’s of exceptions could happen and the paper doesn’t present all the solutions. This is the most important event and also the hardest part to implement. I spent a lot of time in debugging the algorithm of handling this event. 

The software implements about 70% of the paper: the UI , the main sweep algorithm, the Generalized Intersection Event and the Edge Direction Event. The profile offset and anchor events are not implemented yet. These two events are used to model the overhanging roof and decorations.


## Reference:
[1] Kelly T. AND WONKA P. 2011. Interactive Architectural Modeling with Procedural Extrusions. ACM Trans. Graphics, 30, 2, 1- 15.
