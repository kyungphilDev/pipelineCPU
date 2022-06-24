### Design pipelineCPU_Cache with verilog

한 학기에 걸쳐 Computer Architecture에 대해 배우며, verilog환경에서 Pipeline CPU를 디자인하고 구현한 프로젝트 입니다.  

### 1. Pipeline_CPU_Data Forwarding + 2bit Global Prediction(Saturation counter)
첫번째 프로젝트 파일은 Data Forwarding과 Global Prediction으로 구현한 Pipeline TSC CPU 입니다.

- Design Diagram
<p align="center">
      <img src="https://user-images.githubusercontent.com/80669616/142618372-3a639914-fdc0-453a-945d-d14491141317.png" width="800"><br>Pipeline TSC CPU Design Architecture Diagram
</p>

### 2. Pipeline TSC CPU + 2-way associative, single level Cache
첫번째 Pipeline CPU를 baseline CPU로 하여 2-way associative, single Cache를 완벽하게 구현하는 것을 목표로 프로젝트를 진행하였다.  
테스트를 돌릴 경우 일부 오류가 발생하는 문제가 있었고, 완벽하게 디버깅을 하진 못하였다. 추후 디버깅할 예정이다.

