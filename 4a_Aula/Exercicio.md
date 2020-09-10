# Exercício em Aula

- Refinar o conhecimento de pubs e subs do ROS
- Aprender os comandos básicos da px4
- Aprender controle do drone offboard com a mavros
- Controlar o drone por posição de way-point (gps disponível)
- **plus** noções de rotação do drone (quaternions)

## Atividade 1

Com uma simulção aberta rodando com o PX4 rode o comando a seguir no terminal da PX4

>  `arm`

Verifica-se que o drone agora esta armado. Rode então o comando

>  `takeoff`

O drone sairá do chão e em seguida entrará em safe mode, primeiro irá subir até uma altura e depois retornará para a posição de origem e irá aterrisar. Isso acontece porque a PX4 considera que a conexão foi perdida, ja que ela recebeu apenas um comando e depois perdeu a conexão.

Para realizarmos as missões vamos utilizar a mavros com o modo de voo offboard. Para tanto devemos analisar o código presente no site da px4

> https://dev.px4.io/v1.9.0/en/ros/mavros_offboard.html

Utilizaremos ele para fazer nosso drone voar pela primeira vez.


## Atividade 2

Sabemos como fazer nosso drone decolar, mas como fazer ele se mover, a ideia é a mesma que usada no tutorial da px4 para subir, mas acrescentar as posições que desajamos ir, façam um quadrado.

## Atividade 3

Vamos descobrir agora como girar o drone, mas para isso precisamos conhecer melhor a mavros

> http://wiki.ros.org/mavros

Dica olhem o tópico que ja estamos publicando e veja sua documentação.
