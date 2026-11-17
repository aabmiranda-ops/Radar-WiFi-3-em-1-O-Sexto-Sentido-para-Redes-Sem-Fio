Radar WiFi 3-em-1: O Sexto Sentido para Redes Sem Fio

> Projeto prático de Segurança da Informação desenvolvido na disciplina de Algoritmos e Estrutura de Dados (CESAR School).

Integrantes da Equipe:
- Alexandre Miranda (aabmiranda@gmail.com / (https://github.com/aabmiranda-ops))
- Bernardo Acioli (bmfca@cesar.school) / https://github.com/aciolibernardo-code


1. O Problema e a Ideia do Projeto:

Em ambientes restritos, como salas de servidores, laboratórios ou escritórios corporativos, encontrar um access point (roteador não autorizado) ou uma rede oculta não é tarefa fácil, afinal, não dá para ver sinal de rádio. Além disso, medir a proximidade física de equipamentos sensíveis geralmente depende apenas de barreiras físicas ou câmeras, sem um indicador prático do nível de aproximação.

A ideia deste projeto foi transformar um ESP32 em uma ferramenta de auditoria de Wi-Fi (2.4 GHz). Ele faz a varredura contínua do ambiente de forma totalmente passiva (só escuta a rede, sem conectar ou atacar nada) e trabalha com três modos: mede a distância de um alvo usando a força do sinal, estima em qual sala você está via fingerprinting e caça redes que tentam esconder o SSID.

2. O que cada nodo faz:

   2.1 Radar de proximidade (RSSI): Localiza um alvo Wi-Fi e mostra a proximidade em uma barra de 0 a 100%. Tem um buzzer que apita mais rápido conforme você se aproxima.

   2.2 Geolocalização indoor: Identifica em qual sala o dispositivo está cruzando os BSSIDs do ambiente com uma lista de referências pré-configuradas no código.

   2.3 Detector de redes ocultas: Se encontrar um Acess Point tentando esconder o SSID, ele joga o BSSID e o canal na tela e acende um LED de alerta.

 
3. Hardware e Stack:

    Placa: ESP32 NodeMCU / DevKit 38 pinos (ESP-WROOM-32).

    Display: OLED 1.3" I2C SH1106 128×64.

    Periféricos: Buzzer ativo, 3 LEDs comuns, 1 botão push-button.

    Ambiente: VS Code + PlatformIO.

    Linguagem: C/C++ (Framework Arduino).

4. Pinout e Conexões:
   Segue no arquivo HTML em anexo.

5. Execução do projeto:

   Abra a pasta do projeto no VS Code (você precisa ter a extensão PlatformIO instalada).

   Conecte o ESP32 na USB.

   Na barra inferior do PlatformIO:

         Clique no ícone de Build para compilar.

         Clique em Upload para gravar na placa.

         Abra o Serial Monitor (em 115200 baud).

   Pressione o botão ligado ao GPIO 23 para trocar de tela e testar os modos.

6. Demonstração:

Vídeo da placa rodando na bancada: [Link aqui]

