# Marker Deck

Suporte de marcadores passivos (*passive marker deck*) para o Crazyflie 2.1
Brushless, usado para rastreamento pelo sistema de captura de movimento
Qualisys (QTM). A geometria assimétrica dos marcadores permite ao QTM definir
um *rigid body* e estimar a posição **e** a orientação do drone.

## Conteúdo desta pasta

Coloca aqui os ficheiros do marker deck:

- `*.stl` — modelo pronto a imprimir (impressão 3D);
- `*.step` / `*.f3d` / fonte CAD — modelo editável (opcional, mas recomendado
  para quem queira alterar a geometria);
- notas de impressão (material, alturas de camada, etc.), se aplicável.

## Impressão e montagem

Ver o manual do projeto (secção do *marker deck*) para o procedimento de
montagem no drone e posicionamento dos marcadores.

## Nota técnica

Esta pasta tem um ficheiro `COLCON_IGNORE` para que o `colcon build` a ignore
— não é um pacote ROS 2, são apenas ficheiros de hardware.
