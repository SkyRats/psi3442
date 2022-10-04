# Aprofundamento em sistemas cyberfísicos

Nesta atividade, iremos explorar aspectos de controle de sistemas dinâmicos, implementação realista de controle PID e a solução do exercício proposto na aula 3.

Nesse texto, os tópicos são introduzidos de maniera breve e direta sendo recomendado ao leitor ou leitora uma leitura mais aprofundada na referência: Controle Automático (Castrucci) -  2 Edição.

## 1. Introdução ao controle de sistemas dinâmicos
### 1.1 O significado de controlar e nomeclatura

A ideia de controle automático é muito antiga, mas popularmente considera-se o regulador de Watt como sendo o primeiro controlador automático da humanidade.

A ideia de controlar consiste em condicionar uma variável de interesse a um valor especificado. Por exemplo, manter o nível de uma piscina em 2.5m de altura ou comandar um drone para que saia de um ponto origem **A** e chege a um ponto destino **B**.

Os primeiros controles, antes do regulador de Watt implementavam ''sistema de malha aberta'' em que para controlar a temperatura de um forno, uma embalagem de Lasanha recomenda que se pré-aqueça um forno por 15 min sem que se meça a temperatura do forno para se saber se de fato a temperatura encontra-se no patamar desejado.

Depois, surgiram os controladores em ''malha fechada'' em que uma medida daquilo que se desejava controlar era repassada para um controlador realizar um esforço de controle com base no erro entre o valor desejado e o valor medido da variável que se deseja controlar. Um exemplo clássico desse sistema é quando alguém desjea tomar um banho quente, mas a água está muito fria e percebendo o erro entre a temperatura desejada da água e a temperatura gélida daquele instante a pessoa fecha um pouco o registro para que a diminuição da vazão implique em uma água mais quente para o banho.

Essa ideia de realimentação do sinal medido, comparação com uma referência que resultasse num erro que por sua vez resultasse numa ação de controle deu origem ao que se chama de ''Malha de controle'' que será mostrada a seguir.

### 1.2 Malha de Controle de sistemas SISO

![px4_sitl_overview](imgs/nivelReal.png) 1a)
![px4_sitl_overview](imgs/nivelAbstrato.png) 1b)
![px4_sitl_overview](imgs/nivelMatematico.png) 1c)

fonte das imagens: [PUC-RIO](https://www.maxwell.vrac.puc-rio.br/24097/elementos.html)

Na figura 1a) Se observa um esquema de controle do nível de um reservatório de água. Existe um Suprimento de água que é controlado por uma vávula de controle. Quando falta água, a válvula se abre permitindo repor a água para manter um nível desejado no reservatório. O nível desejado é medido por um sensor de nível que fornece essa informação a um controlador de nível que por sua vez regula a válvula.

Na figura 1b) O mesmo se observa, mas os elementos físicos são abstratídos em um **diagrama de blocos** que por sua vez chega em seu nível de abstração máxima quando retratado por funções de transferência **C(s)** representando o controlador, **G(s)** represenatdndo a dinâmica da planta (reservatório) e **L(s)** representando a dinâmica do sensor de nível. E com sinais no domínio de laplace **R(s)** sendo a referência de nível desejada, **E(s)** o erro entre o nível desejado e o nível atual (note que é realizada uma operação de comparação entre R(s) e o sinal que sai de L(s) comumente denominado B(s)), e **Y(s)** sendo o sinal medido que se deseja controlar. O sinal que sai do bloco C(s) e direciona-se a controlar o bloco G(s) é comumente denominado U(s) e é chamado esforço de controle.

Dado essa maneira de pensar um controlador, o algorítimo de controle **C(s)**  mais utilizados em sistemas SISO *Single Input - Single Output* é o PID *Proporcional Integral e Derivaitivo* que funciona por meio das operações de proporcionalidade, integral e derivada do erro entre o que se tem de referência para uma dada variável e seu valor real em um determinado instante. A seguir, explora-se as características e implementações desse algorítimo.

## 2. O controle PID

A ideia do controle PID é gerar um esforço de controle u(t) a partir de operações sobre erro e(t) ambos mostrados na ''Malha de Controle''.

A expressão linear idelaizada desse algorítimo é exposta a seguir

$u(t) = K_p \times ( e(t) + \cfrac{1}{T_i} \int_{0}^{t} e(\tau) d\tau + T_D \cfrac{de(t)}{dt})$

ou

$u(t) = u_P(t) + u_I(t) + u_D(t)$

O que no domínio de Laplace se escreve como

$\cfrac{U(s)}{E(s)} = K_p \times ( 1 + \cfrac{1}{s \times T_i} + T_D \times s)$

Um aspecto interessante que se deve ter em conta é que muitas vezes um ajuste suficientemente bom de um controlador PID pode ser conseguido, a depender do contexto, com versões simplificadas desse algorítimo, ou seja, utilizando-se somente o controle P ou PI ou PD. A ausencia de uma das letras da sigla PID significa que a parcela ausente não influencia no esforço de controle u(t). 

### 2.1 Proporcional

Essa parcela é intuitiva, quanto maior o erro entre o valor de referência e o valor atual de uma variável, maior deverá ser o eforço de controle u(t) para que o erro e(t) seja minimizado. Por exemplo, se a água do chuveiro estiver muito gelada, então um banhista deverá fechar bastante o registro para que ela saia quente. Por outro lado, caso a água já esteja morna, um leve ajuste no registro será suficiente. Matemáticamente:

$u_P(t) = K_p \times e(t)$

#### Parâmetro K_p
Esse parâmetro indica a agressividade do controle. Ele determina o quão intenso será o esforço de controle u(t) para um determinado valro de erro e(t).

Fazendo-se uma análise mais avançada do lugar das raízes (LGR) pode-se notar que há casos em que o aumento do $K_p$ implica no estabilização de um sistema e que no caso de sistemas de fase não mínima, é comum que o aumento do ganho $K_p$ além de um valor crítico desestabilize o sistema a ser controlado.

E sempre aumentar o $K_p$ resulta num sistema que rastreia uma referência mais rapidamente pois aumenta a intensidade do esforço de controle.

### 2.2 Integral

Essa parcela é muitas vezes introduzida no controle com os seguintes objetivos
* Aumentar o tipo do sistema, o que implica no rastreamento com erro nulo a trajetórias de grau mais elevado
* Acelerar a dinâmica do controle, isto é, diminuir o tempo nescessário para chegar ao regime permanente no valor de referência desejado

A parcela responsável por fazer isso é

$u_I(t) = K_p \times \cfrac{1}{T_i} \int_{0}^{t} e(\tau) d\tau$

E como se vê o erro é integrado de tal maneira que o esforço de controle aumenta se mais tempo passa sem que o erro seja diminuido. Essa parcela do controle aumenta a velocidade do rastreamento da referência pois penaliza o controle caso ele demore muito para atingir o objetivo por meio do aumento do esforço de controle.

Quanto ao tipo do sistema, intuitivamente pode-se imaginar o caso em que se deseje manter um dorne a 1m de altura. Caso um controle proporcional controlasse o empuxo nas hélices desse drone, no momento em que o objetivo fosse alcançado o erro seria nulo e proporcionalmente o empuxo também seria nulo levando a queda do drone. Mas, se houver um valor integrado (acumulado) de erro por meio de uma parcela integrativa do erro, então o esforço de controle com erro nulo seria $u(t) \neq 0$ de tal maneira que haveria empuxo o suficiente para que o drone chegasse na cota 1m. Ou seja, a parcela integral aumenta o típo do sistema pois permite rastrear uma referência que seria impossível de se rastrear somente com controle proporcional.

Essa ideia de integração é utilizada em inumeras outras técnicas de controle além do PID.

É importante salientar contudo que o como se vê na figura, a integração de um erro cosntante resulta em $u_I(t) -> \infty$ quando $t -> \infty$ o que certamente é impraticavel dado que atuadores reais não possuem esforço de controle infinito. Por exemplo, um pistão não consegue impor força infinita sobre um gás numa bomba de encher bicicletas. Logo, não é possível implementar na prática essa versão do integrador linear. Uma versão mais realista será discutida a seguir.

#### Parâmetro T_i

O significado desse parâmetro é que T_i é o tempo nescesário para que, mantido o erro e(t) constante, a integral do erro resulte nesse patamar de erro, isto é:

$e(T_i) = \int_{0}^{T_i} e(\tau) d(\tau)$

A figura a seguir ilustra graficamente o significado dessa equação

![px4_sitl_overview](imgs/acaointegral.png)

Fonte: [Apostila de Controle - Escola Politécnica da Universidade de São Paulo](https://edisciplinas.usp.br/mod/resource/view.php?id=123526)

### 2.3 Derivativo


Essa parcela é muitas vezes introduzida no controle com os seguintes objetivos
* Aumentar a estabilidade do controle
* Permitir ganhos integrativos e proporcionais maiores, o que implica em um controle mais rápido
* Reduzir sobressinal
* Carater preditivo

A parcela responsável por fazer isso é

$u_D(t) = K_p \times  T_D \cfrac{de(t)}{dt}$

E como se vê o erro é derivado de tal maneira que maiores variações do erro geram maior esforço de controle u(t). Ou seja, no início em que o erro varia mais rapidamente esse efeito é potencializado por essa parcela derivativa.

A parcela derivativa está associada a um aumento da estabilidade do sistema por ser um efeito oposto ao integrador. Porém, a parecela derivativa nunca é adicionada da maneira exposta acima pois dessa maneira ela representa um sistema não causal, dado que sua resposta depende de um valor futuro do erro. E ainda, a derivada potencializa o efeito de ruídos de alta frequência dado que a derivada em laplace é um filtro que introduz ganho em altas frequências. Ou seja, um derivador puro pode levar um ruído de ganho baixo para uma saída de amplitude infinita quebrando assim a estabilidade BIBO *Bounded Input - Bounded Output*.

Por outro lado, considerando-se o carater preditivo visto na figura a seguir, a parecela derivativa permite antever o que ocorrerá no futuro próximo permitindo assim que os ganhos proporcional $K_p$ e integral $T_i$ sejam mais agressivos pois quando a partir do momento que o erro tende a zero, a parecela derivativa é capaz de antever esse momento e já compensar os efeitos de ganhos proporcional e integrativo mais elevados do que deveriam ser se somente controle PI. Por esse motivo, a parcela derivativa é capaz de reduzir sobressinal.

#### Parâmetro T_D

O significado desse parâmetro é que T_D é o tempo de horizonte de predição. Matematicamente realiza-se a seguinte aproximação

$e(t+T_D) \approx e(t) + T_D\cfrac{de(t)}{dt}$

em que se T_D é escolhido muito grande, o controle prevê instantes bastante avançados no tempo, mas com baixa precisão. E caso contrário, isto é, com T_D não muito grande, a precisão da previsão aumenta apesar de ser menos avançada no tempo. $T_D = 0$ anula o caráter derivativo do PID tornando-se um controle PI. 

A figura a seguir ilustra graficamente o significado dessa equação

![px4_sitl_overview](imgs/acaodiferencial.png)

Fonte: [Apostila de Controle - Escola Politécnica da Universidade de São Paulo](https://edisciplinas.usp.br/mod/resource/view.php?id=123526)

## 3. Implementação realista do controle PID
### 3.1 Anti-windup

Esse problema ocorre pois controladores mandam um sinal chamado esforço de controle u(t) para um atuador que atua sobre uma planta G(s). E esses atuadores possuem um limite de esforço de controle máximo que são capazes de empregar. Por exemplo, um par motor-hélice consegue prover um empuxo máximo limitado pela rotação máxima do motor escolhido e pelo design da hélice.

### 3.2 Filtro de derivada
### 3.3 Derivada do sinal medido versus derivada do erro
### 3.3 Exemplo de implementação!
## 4. Aplicação: Solução do exercício proposto na aula 3.

