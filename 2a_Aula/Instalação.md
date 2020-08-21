# Instalação do Linux

Nesta disciplina recomendamos o linux como base para rodar o ROS e o Gazebo (se algum aventureiro disser que consegue rodar tudo em windows, sinta-se livre para). Dentre as varias distribuições recomendamos o Ubuntu (18.04 ou 20.04)
ou o Debian (9 ou 10). Particularmente nós monitores utilizamos o Ubuntu 18.04 (em algum momento não muito distante o 20.04), mas qaulquer um destes que você escolher será tranquilo.

Então para poder rodar estes sistemas operacionais você pode instalar o sistema operacional na sua maquina (dual boot ou formatar ela mesmo) (**RECOMENDADO**), ou utilizar uma
máquina virtual ou wsl com o windows 10.

## Dual boot

Aqui vamos paresentar a instalação do Ubuntu, caso deseje fazer de outro OS uma rapida pesquisa você encontra o que deseja. Para fazer um dual boot você vai precisar:

- Um PC
- Um Pendrive

1. Pendrive bootavel

>Siga o [tutorial do próprio ubuntu](https://ubuntu.com/tutorials/create-a-usb-stick-on-windows#1-overview) este
tutorial em resumo vai te mandar baixar a ISO que você deseja (pode ser a 18.04 ou a 20.04 para a disciplina), baixar o rufus; formatar o pendrive com o rufus e instalar a ISO.

2. Perarando o seu PC

>A nossa inteção é particionar o disco, mas recomendo fazer uma desfragmentação do seu HD antes (SSD não há necessidade), após isso você particionará seu disco através do
"crete and format hard disk partition" do prórpio windows (O tamanho da partição depende da sua vontade e disponibilidade, mas no mínimo uns 100Gb se tiver disponivel e quiser usar o linux dps da
disciplina)

3. Instalando o Ubuntu

>Com o seu pendrive bootavel conectado no computador reinicie ele, durante a inicialização seleciona a opção de bootmode para pendrive
(normalmente é F12, mas pode conferir mais detalhes nesta [tabela](https://linuxconfig.org/install-ubuntu-from-usb-18-04-bionic-beaver))

>Seu pendrive bootará um linux com a primeira opção sendo experimentar o ubuntu ou instalar, se desejar experimentar ele para ver como é um linux será iniciado normalmente
porém se você desligar o computador ele sumirá junto com tudo que você fizer nele (caso selecione experimentar terá um incone na area de tralho para instalar o linux)

>A instalação é meio longuinha e sem muitos segredos, então siga qualquer um destes tutoriais ou outros da internet (se desejar abre 2 ou mais para certificar-se que esta tudo certo):
[1]


