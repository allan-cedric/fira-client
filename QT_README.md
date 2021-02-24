### Build do fira-client/ssl-Client usando um Qt Project (ssl-Client.pro)

* Adicione o submódulo dos clientes: `git submodule update --init --recursive`
* Execute os seguintes passos para obter os arquivos necessários desse submódulo:
```sh
cd FIRAClient/
mkdir build
cd build/
qmake ..
```

* Após isso, vamos compilar o projeto principal:
```sh
cd ../ssl-Client
mkdir build
cd build/
qmake ..
make
```

* O Makefile do diretório `fira-client/ssl-Client/build/` se torna o **arquivo principal de compilação**.

* O executável(`ssl-Client`) se encontra no diretório `fira-client/ssl-Client/bin/`.