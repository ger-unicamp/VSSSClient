# GER Very Small Size Soccer - Remote Edition 🦠

Repositório de código direcionado à competição na categoria de Very Small Size Soccer(VSSS) produzido pelo Grupo de Estudos em Robótica da Unicamp(GER).

## Pré-requisitos
Para compilar os códigos corretamente, é necessário blablabla firasim blablabl.
Para evitar conflitos de versoes e blablabla, você pode usar o Docker

### Docker
Baixe o docker e faça num sei o q

## Compilação
Para compilar o código, blablabla

## Execução
Para executsar o código blablabla

## Introdução
O código é dividido nas seguintes das seguintes formas

O diagrama da modelagem ta ae

### net

Contém os métodos que conectam o FiraSim ao nosso código pela rede. A conexão é estabelescida utilizando UDP, como um broadcast, sem confirmação da recepção das mensagens. 

### pb

Protocol Buffers (Protobuf) é um método de serialização de dados estruturados. É útil no desenvolvimento de programas que se comunicam uns com os outros ou para armazenar dados. É uma das alternativas ao XML/JSON. A pasta **pb** contém arquivos que definem os protobufs utilizados no projeto.

A pasta **proto** descreve as mensagens enviadas e recebidas entre o FiraSim e nosso código. Cada mensagem tem uma estrutura diferente documentada em sua própria pasta. 

Exemplos de mensagens são:

- Enviar ao FiraSim informações sobre o reposicionamento dos robôs após uma falta.
- Receber a posição da bola e dos robôs no campo.
- Intervenções do juiz no jogo.

### strategy

Contém métodos uteis que definem a estratégia do jogo, além de métodos adicionais para detectar objetos no campo e iniciar partidas.

A estratégia se utiliza de *univector potential fields*, basicamente campos vetoriais que guiam os robôs até a bola, desviando de obstáculos. A bola emite um campo atrativo, enquanto os robôs emitem campos repulsivos, para evitar colisões.

<img src="doc/field.png" alt="alt text" width="200px">

O processo é melhor descrito no seguinte paper:
[Univector Field Navigation With Collision Avoidance](doc/UnivectorFieldNavigationWithCollisionAvoidance.pdf)

### train

Contém arquivos relacionados ao algoritmo genérico utilizado para otimizar os parâmetros do campo vetorial.

- **TODO:** Descrever os cromossomos
- **TODO:** Descrever fitness function
- **TODO:** Descrever como treinar

O algoritmo evolucionário também é descrito no paper:
[Univector Field Navigation With Collision Avoidance](doc/UnivectorFieldNavigationWithCollisionAvoidance.pdf)

### util

Contém métodos gerais que sejam úteis.

- **argparse:** trata os argumentos passados por linha de comando.
- **mathutil:** métodos matemáticos úteis.
- **timer:** define métodos para o timer global.
- **util:** outros métodos gerais. Funções matemáticas, operações bitwise, cálculo com ângulos, estruturas de dados, acesso à memória, entre outros.
- **vec2:** representa o tipo vec2, para vetores no espaço ℝ<sup>2</sup>. Define operações vetoriais, como produto escalar e normalização.





