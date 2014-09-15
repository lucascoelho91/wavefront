//============================================================================
// Name        : graphBuilder.cpp
// Author      : Lucas Coelho Figueiredo
// Version     : 2.0
// Copyright   : If you get money from using this code, you owe me money. If not, it's free to use
// Description : This program builds a map/graph using a pgm file as input
//============================================================================

#include <wavefront/graphBuilder.h>

int coord(int x, int y) //dá o valor do vizinho, de 0 a 7, sendo 0 ao norte, 1 ao nordeste, etc.
{
    if(y==1 && x==-1) return 7;
    else if(y==1) return (x);
    else if(y==-1) return(4-x);
    else if(y==0) return(4-2*x);
    else return 0;
}

double graph::getSquareSize(){
    return this->squareSize;
}

double graph::getSizeMetersPixel(){
    return this->sizeMetersPixel;
}

node* graph::getNodeByIndex(int x, int y)
{
    return matrixGraph[y][x];
}

node* graph::getNodeByPose(Vector2 p)
{
    int x, y;
    //printf("pose: x: %f, y: %f\n", p.x, p.y);
    //printf("discr: %f, resolution: %f\n", squareSize, sizeMetersPixel);
    x = (int) p.x/(squareSize*sizeMetersPixel);
    y = (int) p.y/(squareSize*sizeMetersPixel);
    //printf("cell >>> x: %d, y: %d\n", x, y);
    return matrixGraph[y][x];
}

void graph::openMapFile(char* file, char* outFile)
{
    mapFile = fopen(file, "r");
    outmap = fopen(outFile, "w+");
}

void graph::readMap(uint8_t threshold)
{
    int i, j, k;
    uint8_t c;
    for(i=0; i<dim.y; i++)
    {
        for(j=0; j<dim.x; j++)
        {
            fscanf( mapFile, "%hhu", &c);   //CAPTURA DA INFORMAÇÃO

            image[i][j].r=c;     //COPIA O VALOR DA COR PARA A MATRIZ DE CORES, UTILIZADA PARA VISUALIZAÇÃO DO GRAFO
            image[i][j].g=c;
            image[i][j].b=c;
        }

    }
    fclose(mapFile);


    for(i=0, j=(dim.y-1); i<dim.y; i++, j--)
    {
        for(k=0; k<dim.x; k++)
        {
            c=image[j][k].r;
            visualization[i][k].r=c;
            visualization[i][k].g=c;
            visualization[i][k].b=c;

            colors[i][k].r=c;
            colors[i][k].g=c;
            colors[i][k].b=c;
            if(c<threshold)             
            {
                matrixGraph[(int) i/squareSize][(int) k/squareSize]=NULL;              //CÉLULA OCUPADA SENDO ANULAD
            }
        }
    }

    for(i=0;i<dim.y; i++)
    {
        free(image[i]);
    }
    free(image);
}

void graph::printFile()
{
    fprintf(outmap, "P6\n");   //P6 SIGNIFICA CODIFICAÇÃO CRU, COLORIDO, ARQUIVO PPM
    fprintf(outmap, "%d %d\n", (int) dim.x, (int) dim.y);
    fprintf(outmap, "255\n");  //ESSE NUMERO REPRESENTA O NUMERO MÁXIMO QUE REPRESENTA A COR. DESSA FORMA, 255 255 255 SIGNIFICA BRANCO
    for(int i=(dim.y-1); i>=0; i--)
    {
        for(int j=0; j<dim.x; j++)
        {
            fprintf(outmap, "%c", visualization[i][j].r);
            fprintf(outmap, "%c", visualization[i][j].g);
            fprintf(outmap, "%c", visualization[i][j].b);
        }
    }
    //printf("Vertices gerados: %ld  Arestas: %ld\n\n\n", num_vertices, num_arestas);
    fclose(outmap);
}

void graph::DrawSquare(int size, int i, int j, rgb pixel)
{
    int x, y, k, l;
    if(size==0)
    {
        visualization[i][j]=pixel;
        return;
    }

    for(y = i-size, k=0; k<size*2 && y<dim.y; k++, y++)
    {
        for(x = j-size, l=0; l<size*2 && x<dim.x; l++, x++)
        {
            visualization[y][x]=pixel;
        }
    }
}

void graph::clearGraph()
{
    for(int i=0; i<vertices.y; i++)
    {
        for(int j=0; j<vertices.x; j++)
        {
            if (matrixGraph[i][j]!=NULL)
            {
                matrixGraph[i][j]->powerDist = HUGE_VAL;
                matrixGraph[i][j]->s = NULL;
            }
            
        }
    }
}

void graph::verticesAllocation()
{
    int i, j;
    node* n;
    rgb pixelred, pixelblue;
        pixelred.r=255;
        pixelred.g=0;
        pixelred.b=0;

        pixelblue.r=0;
        pixelblue.g=0;
        pixelblue.b=255;

    for(i=0; i<vertices.y; i++)
    {
        for(j=0; j<vertices.x; j++)
        {

            if(matrixGraph[i][j]!=NULL)     //SE NÃO É NULO, A CÉLULA PASSOU PELO TESTE E VAI RECEBER UM ELEMENTO
            {
                matrixGraph[i][j]=(node*) malloc(sizeof(node));
                n=matrixGraph[i][j];
                //n->occupiable=1;
                n->pose.x= sizeMetersPixel*(j*squareSize + squareSize/2);
                n->pose.y= sizeMetersPixel*(i*squareSize + squareSize/2);

                //COLORIR A IMAGEM DE SAÍDA COM UM PONTO AZUL ONDE HÁ UMA CÉLULA OCUPÁVEL
                DrawSquare(1, i*squareSize + squareSize/2 , j*squareSize + squareSize/2 , pixelblue);
            }
            else
            {
                //SE É NULO, NÃO É ALOCADO UMA CÉLULA E A IMAGEM RECEBE UM PONTO VERMELHO NO LOCAL DA CÉLULA

                DrawSquare(1, i*squareSize + squareSize/2 , j*squareSize + squareSize/2 , pixelred);
            }
        }
    }
}


void graph::connectNeighbors()
{
    rgb pixelgreen;
    pixelgreen.r=0;
    pixelgreen.g=255;
    pixelgreen.b=0;
    int i, j, k, l, x, y, c, BLOCK = 2;
    for (i=0; i<vertices.y; i++)
    {
        for(j=0; j<vertices.x; j++)
        {
            if(matrixGraph[i][j]!=NULL)
            {
                for(k=-1; k<=1; k++)
                {
                    for(l=-1; l<=1; l++)
                    {
                        if((i+k)>=0 && (j+l)>=0 && (i+k)<vertices.y && (j+l)<vertices.x)
                        {
                            x=coord(k,l);  //CALCULA O NÚMERO DO VIZINHO DO VÉRTICE
                            printf("%d\n", x);
                            if (matrixGraph[(i+k)][(j+l)]!=NULL)
                            {
                                matrixGraph[i][j]->neighbor[x]=matrixGraph[i+k][j+l];

                                    //DANG! 8 FORS ONE INSIDE ANOTHER? IS THIS TRIP REALLY NECESSARY? https://www.youtube.com/watch?v=utS4m6I8SCM

                            }
                            else
                            {
                                matrixGraph[i][j]->neighbor[x]=NULL;
                            }
                        }
                    }
                }
            }
        }
    }


    for (i=0; i<vertices.y; i++)
    {
        for(j=0; j<vertices.x; j++)
        {
            if(matrixGraph[i][j]!=NULL)
            {
                for(k=-1; k<=1; k++)
                {
                    for(l=-1; l<=1; l++)
                    {
                        if (matrixGraph[(i+k)][(j+l)]!=NULL)
                        {
                            for(y=(i*squareSize+squareSize/2+k*2), x=(j*squareSize+squareSize/2+l*2), c=0; c< (squareSize-2*BLOCK); c++, x=x+l, y=y+k)
                            {
                                visualization[y][x]=pixelgreen;
                            }
                        }
                    }
                }
            }
        }
    }
}

graph::graph (double sizeDiscretization, char* file, double sizeMetersPixel, char* outFile)
{
    int i, j;
    openMapFile(file, outFile);

    squareSize = sizeDiscretization;
    this->sizeMetersPixel = sizeMetersPixel;
    printf("squareSize %f\n", squareSize);
    char* str = (char*) malloc(100*sizeof(char));
    //OS FGETS CAPTURAM AS INFORMAÇÕES QUE NÃO SERÃO UTILIZADAS NO PROGRAMA


    fgets(str, 100, mapFile);    //AQUI É CAPTURADO O TIPO DE CODIFICAÇÃO
    fgets(str, 100, mapFile);    //AQUI A LINHA DE COMENTÁRIO QUE O GIMP DEIXA

    fscanf (mapFile, "%lf", &dim.x);
    fscanf (mapFile, "%lf", &dim.y);


    fgets(str, 100, mapFile);   //AQUI O VALOR MÁXIMO DE CADA BYTE
    fgets(str, 100, mapFile);   //AQUI UM \N QUE INDICA O COMEÇO DA IMAGEM

    vertices.y = dim.y/squareSize;
    vertices.x = dim.x/squareSize;


    /***** ALOCAÇÃO DAS VARIÁVEIS PRINCIPAIS ******************/

    node vertexToPoint; //É UMA VARIÁVEL node UTILIZADA SOMENTE PARA NÃO APONTARMOS PARA UMA COISA QUE NÃO SEJA UM node
                            //TODAS OS VÉRTICES ALOCADOS DE node*** graph APONTARÃO PRA vertexToPoint

    colors=(rgb**) malloc(dim.y*sizeof(rgb*));
    for(i=0; i<dim.y; i++)
    {
        colors[i]=(rgb*) malloc(dim.x*sizeof(rgb));
    }

    visualization=(rgb**) malloc(dim.y*sizeof(rgb*));
    for(i=0; i<dim.y; i++)
    {
        visualization[i]=(rgb*) malloc(dim.x*sizeof(rgb));
    }

    for(i=0;i<=vertices.y;i++)
    {
        std::vector <node*> row;
        for(j=0; j<=vertices.x; j++)
        {
            row.push_back(&vertexToPoint);
        }
        matrixGraph.push_back(row);
    }

    image=(rgb**)malloc(dim.y*sizeof(rgb*));
    for(i=0; i<dim.y; i++)
    {
        image[i]=(rgb*)malloc(dim.x*sizeof(rgb));
    }

}

void graph::BuildGraph(int threshold=250)
{
    readMap(threshold);
    verticesAllocation();
    connectNeighbors();
    printFile();
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "PID");
    ros::NodeHandle n;
/*    if(argc != 6)
    {
        printf("Invalid arguments. Usage: \\.graphBuilder < pgm map file > < discretization > < map resolution >");
        return 1;
    }

    char mapFile[255];
    strcpy(mapFile, argv[1]);
    printf("%s\n", argv[1]);
    assert(mapFile != NULL && strcmp(mapFile, "\0"));

    double discretization = strtod(argv[2], NULL);
    assert(discretization != 0.0 && errno!=ERANGE);
    printf("%s\n", argv[2]);

    double resolution = strtod(argv[3], NULL);
    assert(resolution != 0.0 && errno!=ERANGE);
    printf("%s\n", argv[3]);

    int threshold = (int) strtol(argv[4], NULL, 10);
    assert(threshold != 0 && errno!=ERANGE);
    printf("%s\n", argv[4]);

    char outFile[255];
    strcpy(outFile, argv[5]);
    printf("%s\n", argv[5]);
    assert(mapFile != NULL && strcmp(outFile, "\0"));
*/
        
    
    double discretization, resolution;
    int threshold = 250;
    std::string mapFile, outFile;

    if(ros::param::get("/wavefront/discretization", discretization));
    else{
        ROS_INFO("Error getting parameter square size parameter");
        exit(1);
    }
    if(ros::param::get("/wavefront/resolution", resolution));
    else{
        ROS_INFO("Error getting parameter: resolution");
        exit(1);
    }
    if(ros::param::get("/wavefront/map",mapFile));
    else{
        ROS_INFO("Error getting parameter: map file.");
        exit(1);
    }
     if(ros::param::get("/wavefront/outmap",outFile));
    else{
        ROS_INFO("Error getting parameter: out file.");
        exit(1);
    }
    /*if(ros::param::get("/wavefront/robotsConfFile",robotsConfFile));
    else{
        ROS_INFO("Error getting parameter: robots configuration file.");
        exit(1);
    }*/

  


    graph Grafo(discretization, (char*) mapFile.c_str(), resolution, (char*) outFile.c_str());



    Grafo.BuildGraph(threshold); 

    return 0;

}
