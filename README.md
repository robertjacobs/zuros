ZUYD university ZUROS robot platform
---

The development of robot applications intended to support care provision requires a multidisciplinary approach. In the conception, development and evaluation of any robot application both the technical developers and the care providers and end users should be involved. This also is valid when it comes to higher education concerning care and technological support for care. 

To make this possible a physical meeting point bringing together robot development knowledge and care delivery knowledge should come together. This thesis described the build-up of a conceptual robot platform as a basis for care application development to be embedded into the education of students connected to EIZT, the Expertise center for Innovative Care and Technology, enabling students from different disciplines to collaborate. 

More concrete, the research center Technology in care of ZUYD University of technology invested in a robot platform to be used in the context of care research and related education. Its purpose is to provide a basic platform onto which a variety of applications could be build. This platform is meant to function primarily at the lab situation located in the building of Zorg Acedemie Parkstad, Heerlen. The selected platform is a platform developed at the robotics center of the faculty of 3mE at TU Delft. It is the same platform used for the minor courses at TU Delft. 

__Project is structured as follows:__ 

__zuros_sensors:__ 
Package for reading external sensors (environental sensors like zwave and robot sensors like asus xtion) and sending their information to other packages using ROS

__zuros_sequencer:__ 
Package which contains sequencing components (like driving around)

__zuros_deliberator:__ 
Package which contains deliberation components (like navigation)

__zuros_control:__ 
Package which contains control component (like motor driver)
