# Instalação do Linux

Nesta disciplina recomendamos o linux como base para rodar o ROS e o Gazebo (se algum aventureiro disser que consegue rodar tudo em windows, sinta-se livre para). Dentre as varias distribuições recomendamos o Ubuntu (18.04 ou 20.04)
ou o Debian (9 ou 10). Particularmente nós monitores utilizamos o Ubuntu 18.04 (em algum momento não muito distante o 20.04), mas qaulquer um destes que você escolher será tranquilo.

Então para poder rodar estes sistemas operacionais você pode instalar o sistema operacional na sua maquina (dual boot ou formatar ela mesmo) (**RECOMENDADO**), ou utilizar uma
máquina virtual ou wsl com o windows 10.

## Dual boot

ANTES DE PROSSEGUIR LEIA **TUDO** QUE ESTA AQUI E ALGUNS TUTORIAS NA INTERNET (recomendo a leitura deste aqui [tutorial](https://www.itzgeek.com/how-tos/linux/ubuntu-how-tos/how-to-install-ubuntu-18-04-alongside-with-windows-10-or-8-in-dual-boot.html)) 

Aqui vamos paresentar a instalação do Ubuntu, caso deseje fazer de outro OS uma rapida pesquisa você encontra o que deseja. Para fazer um dual boot você vai precisar:

- Um PC
- Um Pendrive

1. Pendrive bootavel

>Siga o [tutorial do próprio ubuntu](https://ubuntu.com/tutorials/create-a-usb-stick-on-windows#1-overview). Este
tutorial em resumo vai te pedir para:
* Baixar a ISO que você deseja (pode ser a 18.04 ou a 20.04 para a disciplina)
* Baixar o Rufus
* Formatar o pendrive com o rufus e instalar a ISO. 
**Obs**: O Rufus ja vai formatar e "instalar" a ISO no seu Pen Drive

2. Perarando o seu PC

>A nossa inteção é particionar o disco, mas recomendo fazer uma desfragmentação do seu HD antes (SSD não há necessidade), após isso você particionará seu disco através do "create and format hard disk partition" do prórpio windows (O tamanho da partição depende da sua vontade e disponibilidade, mas no mínimo uns 100Gb se tiver disponivel e quiser usar o linux dps da disciplina)

3. Instalando o Ubuntu

>Com o seu pendrive bootável conectado no computador reinicie ele, durante a inicialização seleciona a opção de bootmode para pendrive
(normalmente é F12, mas pode conferir mais detalhes nesta [tabela](https://linuxconfig.org/install-ubuntu-from-usb-18-04-bionic-beaver))
**Obs**: As teclas para entar no Boot Selection do seu computador também podem ser F2 ou F10, porém isto depende da fabricante do modelo da sua placa mãe

>Seu pendrive bootará um linux com a primeira opção sendo experimentar o ubuntu ou instalar, se desejar experimentar ele para ver como é um linux será iniciado normalmente
porém se você desligar o computador ele sumirá junto com tudo que você fizer nele (caso selecione experimentar terá um incone na area de tralho para instalar o linux)

>Na parte de escolher como instalar o ubuntu ela é bem delicada, a opção de instalar ao lado do windows é a mais simples, mas você não consegue manejar os espaços de disco usado
o instalador fará tudo para você. Se quiser ter maior controle, como por exemplo escolher quanto de memória swap irá colocar o mais recomendado é a opção "something else" que você pode manejar as partições das suas memórias (porém é meio delicado, você não vai querer selecionar a partição errada, então tome cuidado)

## WSL

O WSL é uma ferramenta do próprio Windows 10 que permite rodar um terminal de linux no seu computador. É bem prático de usar, apesar de ter alguns macetes para funcionar adequadamente, além de que é um pouco perigoso, teoricamente era para ele ser isolado do windows, mas eu consegui acessar todo o sistema do windws.

1. [Para habilitar o WSL basta seguir o tutorial do próprio windows](https://docs.microsoft.com/pt-br/windows/wsl/install-win10)

2. Abre a microsoft store e procure a distro q você deseja, baixe ela e digite o nome da distro na barra de busca do windows, pronto você tem um terminal da distro de linux que você quer aberto.

3. o WSL é apenas um terminal, então para rodar janelas você vai precisar fazre umas gambiarras, primeiro faça o [download do Xming](https://sourceforge.net/projects/xming/)

4. siga esse [tutorial que faz você instalar o openssh](https://virtualizationreview.com/articles/2017/02/08/graphical-programs-on-windows-subsystem-on-linux.aspx)

Caso com o Xming aberto no windows e rodar o xeyes no wsl não aparecer os olhos no Xming entre em contato.


## Dicas Básicas de Linux

Se você está começando a se aventurar neste universo de distribuições Linux e está se sentindo meio perdido, recomendo a leitura de um documento parecido com este tutorial aqui com dicas valiosas para quem está começando agora.

O link do documento que preparamos para um Workshop sobre ***Intro to Linux*** se encontra [neste link](https://github.com/SkyRats/workshops/tree/linux_basics).