# FEM elastic rod 

- [x] Explicit taichi implementation modified from taichi example `fem99.py`.
- [x] Low-performance Implicit FEM rod C++ implementation in reference of http://www.femdefo.org/ with Eigen.
- [x] Neo-hookean material model

## INSTALL

```
msbulid FEM_rod.sln /p:configuration=Release && x64\Release\FEM_rod.exe
```

```
python fem.py
```

## Demo
1. explicit

![](assets/explicit_8_8_x2.gif)

2. implicit
   
![](assets/implicit_8_8.gif)