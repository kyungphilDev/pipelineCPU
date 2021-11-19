### Design pipelineCPU_Cache with verilog

verilog환경에서 Pipeline CPU를 디자인하고 구현한 프로젝트 입니다.  

### Pipeline_CPU_Data Forwarding + 2bit Global Prediction(Saturation counter)
첫번째 프로젝트 파일은 Data Forwarding과 Global Prediction으로 구현한 Pipeline TSC CPU 입니다.

-Design Diagram
<p align="center">
      <img src="https://user-images.githubusercontent.com/80669616/142618372-3a639914-fdc0-453a-945d-d14491141317.png" width="1000"><br>Pipeline TSC CPU Design Architecture Diagram
</p>

### Pipeline TSC CPU + 2-way associative, single level Cache
첫번째 Pipeline CPU를 baseline CPU로 하여 2-way associative, single Cache를 완벽하게 구현하는 것을 목표로 프로젝트를 진행하였다.  
테스트를 돌릴 경우 일부 오류가 발생하는 문제가 있었고, 완벽하게 디버깅을 하진 못하였다. 굉장히 어려운 프로젝트라고 여겨진다.
