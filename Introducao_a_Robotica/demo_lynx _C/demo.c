#include <stdio.h>
#include <stdlib.h>

#include "ufrn_al5d.h"

//Posicao inicial para todos os servos
#define HOME_POS "#0P1528S500#1P1468S500#2P1672S500#3P1504S500#4P1870S500T2000"
#define STANDBY "#0P1528S500#1P1428S500#2P968S500#3P1504S500#4P1870S500T2000"
#define WAKEUP "#1P1428S500#2P968S500#3P1504S500#4P1870S500T2000"
#define RELAX "#0P0000S2000#1P0000S2000#2P0000S2000#3P0000S2000#4P0000S1000"
#define BACK "#0P1528S2000#1P1600S2000#2P1768S2000#3P1324S2000#4P1870S2000"
#define POS1 "#0P1532S1000#1P1088S1000#2P1612S1000#3P844S1000#4P1990S1000"
#define POS2 "#0P1288S1000#1P1344S1000#2P2060S1000#3P1008S1000#4P1990S1000"
#define POS3 "#0P1832S1000#1P1344S1000#2P2060S1000#3P1008S1000#4P1990S1000"
#define POS4 "#0P1532S1000#1P1088S1000#2P1612S1000#3P844S1000#4P1990S1000"
#define POS5 "#0P1640S1000#1P1180S1000#2P1808S1000#3P836S1000#4P1990S1000"
#define POS6 "#0P1476S1000#1P1180S1000#2P1808S1000#3P836S1000#4P1990S1000"
#define PI 3.14159265
#define px 0
#define py 0
#define OFFSETX 100
#define OFFSETY 200
#define offsetBas 664
#define offsetShl 680
#define offsetElb 2428
#define offsetWri 588
#define gainBas 0.1041667
#define gainShl 0.1142132
#define gainElb 0.1209677
#define gainWri 0.0982533



int serial_fd;
char *comando, *last_comando;

int relax_flag = 0;

char c = 'a';

float X, Y, Z;
float x, y, z, phi = 0.0;

unsigned int pos = 1500;
float angSenseBas = -90.0;
float angSenseShl = 90.0;
float angSenseElb = -90.0;
float angSenseWri = 0.0;
float teta[4] = {0};
float sense[5] = {1500.0, 1500.0, 1500.0, 1500.0, 1500.0};
//float sense[5] = {1528, 1468, 1672, 1504, 1870};


float L1 = 6.3, L2 = 14.6, L3 = 18.3, L4 = 8.5;

float h = 7.0, t1, t2, t3, t4;
int t = 2000, s = 300;

void send_command(void);
void make_and_send_command(void);
void rad2deg(float *ang);
void move(float x, float y, float z, float phi);
void calc_tetas(float x, float y, float z, float phi);
void calc_senses(void);


int main()
{
    ufrn_header();

    // INICIO DO PROGRAMA DEMO //

    printf("PROGRAMA INTOLERANTS INICIADO\n\n");

    serial_fd = abrir_porta();

    if (serial_fd == -1)
    {
        printf("Erro abrindo a porta serial /dev/ttyS0\nAbortando o programa...");
        return -1;
    }
    else
    {
        printf("Porta serial /dev/ttyS0 aberta com sucesso\n");

        if (configurar_porta(serial_fd) == -1)
        {
            printf("Erro inicializando a porta\n");
            close(serial_fd);
            return -1;
        }

        comando = (char*) malloc(sizeof(char) * BUFSIZE);
        last_comando = (char*) malloc(sizeof(char) * BUFSIZE);

           //////////////////////
            // COMANDO TESTE     //
           //////////////////////
        printf("\nPRIMEIRO COMANDO - POSICAL INICIAL\n");

        sprintf(comando, "%s", STANDBY);


        if (enviar_comando(comando, serial_fd) != -1)
        {
            printf("Enviando de comando com teste de envio: %s\n", STANDBY);
        }
        else
        {
            printf("Problema no envio do comando\nAbortando o programa...");
            return -1;
        }

        printf("Pressione enter para continuar...");
        getchar();
        memset(comando, 0, BUFSIZE);

        sprintf(comando, "%s", HOME_POS);

        if (enviar_comando(comando, serial_fd) != -1)
        {
            printf("Enviando de comando com teste de envio: %s\n", HOME_POS);
        }
        else
        {
            printf("Problema no envio do comando\nAbortando o programa...");
            return -1;
        }

        memset(comando, 0, BUFSIZE);

        /////////////////////
        // COMANDO  PRINCIPAL//
        /////////////////////


        do {

            if(system ("clear"));

            t1 = angSenseBas*PI/180.0;
            t2 = angSenseShl*PI/180.0;
            t3 = angSenseElb*PI/180.0;
            t4 = angSenseWri*PI/180.0;

            X = cos(t1)*(L3*cos(t2 + t3) + L2*cos(t2) + L4*cos(t2 + t3 + t4));

            Y = sin(t1)*(L3*cos(t2 + t3) + L2*cos(t2) + L4*cos(t2 + t3 + t4));

            Z = L1 + L3*sin(t2 + t3) + L2*sin(t2) + L4*sin(t2 + t3 + t4);

            calc_tetas(X, Y, Z, phi);
            ///// CINEMATICA DIRETA ----------------///////////////////////
            printf("A coordenada x do ponto : %.2f \n", X);
            printf("A coordenada y do ponto : %.2f \n", Y);
            printf("A coordenada z do ponto : %.2f \n", Z);
            printf("%s\n", last_comando);
            printf("BASE     -> (Q)ESQUERDA ; (A)DIREITA  | ");
            printf("OMBRO    -> (W)CIMA     ; (S)BAIXO    | \n");
            printf("COTOVELO -> (E)CIMA     ; (D)BAIXO    | \n");
            printf("PUNHO    -> (R)CIMA     ; (F)BAIXO    | \n");
            printf("GARRA    -> (T)FECHAR   ; (G)ABRIR    | \n");
	    printf("REPOUSO  -> (r) \n");
            printf("Digite space para sair\n");
            if(system("/bin/stty raw"));
            c = getchar();
            if(system("/bin/stty cooked"));

            //// COMANDO DO META 1 PARA CINEMATICA DIRETA ////////////
            if (c == 'q') {
                sense[0] = sense[0] + 4;
                pos = sense[0];
                angSenseBas = (offsetBas - sense[0]) * gainBas;
                sprintf(comando, "#%dP%d", BAS_SERVO, trava(BAS_SERVO, pos));
            } else if (c == 'a') {
                sense[0] = sense[0] - 4;
                pos = sense[0];
                angSenseBas = (offsetBas - sense[0]) * gainBas;
                sprintf(comando, "#%dP%d", BAS_SERVO, trava(BAS_SERVO, pos));
            } else if (c == 'w') {
                sense[1] = sense[1] + 4;
                pos = sense[1];
                angSenseShl = (sense[1] - offsetShl) * gainShl;
                sprintf(comando, "#%dP%d", SHL_SERVO, pos * (pos < 1850) + 1850 * (pos >= 1850));
            } else if (c == 's') {
                sense[1] = sense[1] - 4;
                pos = sense[1];
                angSenseShl = (sense[1] - offsetShl) * gainShl;
                sprintf(comando, "#%dP%d", SHL_SERVO, pos * (pos < 1850) + 1850 * (pos >= 1850));
            } else if (c == 'e') {
                sense[2] = sense[2] - 4;
                pos = sense[2];
                angSenseElb = (offsetElb - sense[2]) * gainElb - 180;
                sprintf(comando, "#%dP%d", ELB_SERVO, pos);
            } else if (c == 'd') {
                sense[2] = sense[2] + 4;
                pos = sense[2];
                angSenseElb = (offsetElb - sense[2]) * gainElb - 180;
                sprintf(comando, "#%dP%d", ELB_SERVO, pos);
            } else if (c == 'r') {
                sense[3] = sense[3] + 4;
                pos = sense[3];
                angSenseWri = (sense[3] - offsetWri) * gainWri - 90;
                sprintf(comando, "#%dP%d", WRI_SERVO, trava(WRI_SERVO, pos));
            } else if (c == 'f') {
                sense[3] = sense[3] - 4;
                pos = sense[3];
                angSenseWri = (sense[3] - offsetWri) * gainWri - 90;
                sprintf(comando, "#%dP%d", WRI_SERVO, trava(WRI_SERVO, pos));
            } else if (c == 't') {
                sense[4] = sense[4] + 10;
                pos = sense[4];
                //sprintf(comando, "#%dP%d", GRI_SERVO, trava(GRI_SERVO, pos));
		sprintf(comando, "#4P2300S300T1500");
                enviar_comando(comando, serial_fd);
            } else if (c == 'g') {
                sense[4] = sense[4] - 10;
                pos = sense[4];
                //sprintf(comando, "#%dP%d", GRI_SERVO, trava(GRI_SERVO, pos));
		sprintf(comando, "#4P1500S300T1500");
                enviar_comando(comando, serial_fd);
	    }else if(c == 'u'){
		move(-0.0,-23.81,32.24,35);
            } else if (c == 'p') {
                if(system("gnome-terminal -x /home/aluno/intolerants/meta3/meta3"));
            } else if (c == '=') {
                FILE *f = fopen("input.txt", "a");
                if (f == NULL)
                {
                    printf("Error opening file!\n");
                    exit(1);
                }

                /*
                fprintf(f, "%d %d\n", (int)X+OFFSETX, (int)Y+OFFSETY);
                fclose(f);
                FILE *f2 = fopen("pos.txt", "a");
                if (f2 == NULL)
                {
                    printf("Error opening file!\n");
                    exit(1);
                }
                fprintf(f2, "#0P%uT1000#1P%uT1000#2P%uT1000#3P%uT1000#4P%uT1000\n", (unsigned int)sense[0], (unsigned int)sense[1], (unsigned int)sense[2], (unsigned int)sense[3], (unsigned int)sense[4]);
                 */
                 //fclose(f2);

            ///------------------------- TRAJETORIA CALCULANDO PARA SEGUIR ---------------------///
            }
 /*else if (c == '1') {
             printf("ENTROU AQUI\n");
              move(-13.89,-27.16,14.89,0.0);
              printf("%s\n", comando);
    printf("ponto 1\n");
    //sleep(5);
            //     sprintf(comando, "%s", POS1);
            } else if (c == '2') {
             printf("ENTROU AQUI\n");
              move(2.27,-31.16,14.89,0.0);
              printf("%s\n", comando);
    printf("ponto 1\n");
    }else if (c == '3') {
             move(2.81,-35.07,17.32,0);
    printf("%s\n", comando);
    printf("ponto 2\n");
    //sleep(3);
            //     sprintf(comando, "%s", POS2);
             } else if (c == '4') {
    move(13.49,-33.26,21.10,0);
    printf("%s\n", comando);
    printf("ponto 3\n");
    sleep(3);
}*/

     else if (c == '1') {
             printf("ENTROU AQUI\n");
              move(-14.43,-25.40,13.02,0.0);
              printf("%s\n", comando);
    printf("ponto 1\n");
    //sleep(5);
            //     sprintf(comando, "%s", POS1);8
            } else if (c == '2') {
             printf("ENTROU AQUI\n");
              move(-14.43,-25.40,15,0.0);
              printf("%s\n", comando);
    printf("ponto 1\n");
    }else if (c == '3') {
             move(12.93,-26.22,15,0);
    printf("%s\n", comando);
    printf("ponto 2\n");
    //sleep(3);
            //     sprintf(comando, "%s", POS2);
             } else if (c == '4') {
    move(15.24,-31.08,17.5,0);
    printf("%s\n", comando);
    printf("ponto 3\n");
    sleep(3);
            //     sprintf(comando, "%s", POS3);
             } else if (c == '5') {
    move(12.87,-24.29,1.20,-20);
    printf("%s\n", comando);
    printf("ponto 4\n");
    //sleep(3);
            //     sprintf(comando, "%s", POS4);
             } else if (c == '6') {
    move(4.44,-17.71,1.0,-20);
    printf("%s\n", comando);
    printf("ponto 5\n");
    //sleep(3);
            //     sprintf(comando, "%s", POS5);

            ///-----------------------TERMINAR A TRAJETORIA 1 -----------------------------------------------------------///

            ///-------------------------- TRAJETORIA CALCULANDO 2 -------------------------------------------------------///
            /*
            } else if (c == '1') {
                printf("ENTROU AQUI\n");
                    move(0.0,-22.78,1.70,0.0);
                    printf("%s\n", comando);
                    printf("ponto 1\n");
                    //sleep(5);
            //     sprintf(comando, "%s", POS1);
            } else if (c == '2') {
                move(-2.00,-11.46,1.97,-10);
                    printf("%s\n", comando);
                    printf("ponto 2\n");
                    //sleep(3);
            //     sprintf(comando, "%s", POS2);
             } else if (c == '3') {
                    move(16.03,-5.69,1.70,-20);
                    printf("%s\n", comando);
                    printf("ponto 3\n");
                    sleep(3);
            //     sprintf(comando, "%s", POS3);
             } else if (c == '4') {
                    move(17.90,-24.29,1.20,-20);
                    printf("%s\n", comando);
                    printf("ponto 4\n");
                    //sleep(3);
            //     sprintf(comando, "%s", POS4);
             } else if (c == '5') {
                    move(2.46,-18.92,1.20,-30);
                    printf("%s\n", comando);
                    printf("ponto 5\n");
                    //sleep(3);
            //     sprintf(comando, "%s", POS5);
            */

            // --------------------------------------TERMINA TRAJETORIA 2-----------------------------------------------------//////


            } else if(c=='6'){
             sense[4] = sense[4] - 700;
                pos = sense[4];
                sprintf(comando, "#%dP%d", GRI_SERVO, trava(GRI_SERVO, pos));
            }
            else if(c=='7'){
             sense[4] = sense[4] + 700;
                pos = sense[4];
                sprintf(comando, "#%dP%d", GRI_SERVO, trava(GRI_SERVO, pos));
            } else if(c == '8'){
 sprintf(comando, HOME_POS);
    }else if (c == 'h') {
                printf("Insira s e t: ");
                if(scanf("%d", &s));
                if(scanf("%d", &t));

                sprintf(comando, "#0P1528S%d#1P1468S%d#2P1672S%d#3P1504S%d#4P1870S%dT%d", s, s, s, s, s, t);
            } else if (c == 'z') {

                sprintf(comando, "#0P1540S300#1P1244S300#2P1716S300#3P1720S300#4P1150S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1540S300#1P1244S300#2P1716S300#3P1720S300#4P1160S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1540S300#1P1148S300#2P1596S300#3P1720S300#4P1160S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1532S300#1P1148S300#2P1596S300#3P1720S300#4P1980S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1532S300#1P1232S300#2P1596S300#3P1732S300#4P1980S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1448S300#1P1548S300#2P2144S300#3P1844S300#4P1980S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1448S300#1P1548S300#2P2176S300#3P1916S300#4P1980S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1400S300#1P1364S300#2P1996S300#3P1916S300#4P1980S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
            }



            // CINEMATICA INVERSA --------------- ////////////

            else if (c == 'm'){
                printf("\n\n");
                printf("x: ");
                if(scanf("%f", &x));
                printf("y: ");
                if(scanf("%f", &y));
                printf("z: ");
                if(scanf("%f", &z));
                printf("phi: ");
                if(scanf("%f", &phi));
                move(x,y,z,phi);
                printf("%s\n", comando);
            }

            if (enviar_comando(comando, serial_fd) != -1)
            {
                printf("\nEnviando de comando com teste\n");
            }
            else
            {
                printf("Problema no envio do comando\nAbortando o programa...");
                return -1;
            }

            memset(comando, 0, BUFSIZE);

        } while ( c != ' ');

        sprintf(comando, "%s", STANDBY);
        enviar_comando(comando, serial_fd);
        memset(comando, 0, BUFSIZE);
        printf("Pressione enter para relaxar...");
        getchar();
        sprintf(comando, "%s", RELAX);
        enviar_comando(comando, serial_fd);

        // FIM DO PROGRAMA DEMO //
        fechar_porta(serial_fd);
        printf("\nAcesso a porta serial /dev/ttyS0 finalizado\n");

    }

    printf("\nPROGRAMA FINALIZADO\n\n");

    return 0;
}




 // FUNÇÕES INCLUINDO OPENCV --------------- /////////////
void send_command(void) {
    enviar_comando(comando,serial_fd);
    sprintf(last_comando, "%s", comando);
    memset(comando, 0, BUFSIZE);
}

void make_and_send_command(void){
    sprintf(comando, "#0P%uS500#1P%uS500#2P%uS500#3P%uS500\n", (unsigned int)sense[0], (unsigned int)sense[1], (unsigned int)sense[2], (unsigned int)sense[3]);
    send_command();
}

void rad2deg(float *ang){
    *ang *= 180.0/PI;
}
//

void calc_tetas(float x, float y, float z, float phi) {

    int i;
    phi *= PI/180.0;
    float exy = sqrt(pow(x,2) + pow(y,2));
    teta[0] = atan2(y/exy, x/exy);
    float x14 = exy - L4*cos(phi);
    float z14 = z - L1 - L4*sin(phi);
    float c3 = ((pow(x14, 2) + pow(z14, 2) - pow(L2, 2) - pow(L3, 2))/(2*L2*L3));
    float s3 = -sqrt(1-pow(c3,2));
    teta[2] = atan2(s3,c3);
    float exz14 = sqrt(pow(x14,2) + pow(z14,2));
    float alpha = atan2(z14/exz14, x14/exz14);
    float beta = atan2(sin(teta[2])*L3/exz14, (L2 + L3*c3)/exz14);
    teta[1] = alpha - beta;
    teta[3] = phi - teta[1] - teta[2];
    for (i = 0; i < 4; i++){
        rad2deg(&teta[i]);
}
}

void calc_senses(){
    sense[0] = -(teta[0]/0.1041667 - 636 - 28);
    sense[1] = teta[1]/0.1142132 - 32 + 712;
    sense[2] = -((teta[2]+180.0)/0.1209677 - 2256 - 172);
    sense[3] = (teta[3]+90)/0.0982533 + 4 + 584;
}

/// COMANDO PARA ENVIAR A TRAJETORIA VIA CINEMATICA INVERSA
void move(float x, float y, float z, float phi){
    calc_tetas(x, y, z, phi);
    calc_senses();
    make_and_send_command();
    X = x;
    Y = y;
    Z = z;
    angSenseBas = (offsetBas - sense[0]) * gainBas;
    angSenseShl = (sense[1] - offsetShl) * gainShl;
    angSenseElb = (offsetElb - sense[2]) * gainElb - 180.0;
    angSenseWri = (sense[3] - offsetWri) * gainWri - 90.0;
}

