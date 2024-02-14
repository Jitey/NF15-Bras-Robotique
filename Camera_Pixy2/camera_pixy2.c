#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdio.h> 


int i = 0;
int j = 0;
bool camera = false;
bool front = false;
uint32_t gFPS = 0;


void init_SPI(void)
{
    //------Configuration du mode SPI maître----------------------------------------------------
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Mettre EUSCI_BO en état reset
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MST |  // Mode maître
        EUSCI_B_CTLW0_SYNC | // Mode synchrone (SPI)
        EUSCI_B_CTLW0_CKPL | // Polarité, état inactif à 1
        EUSCI_B_CTLW0_MSB |  // Envoi du MSB en premier
        EUSCI_B_CTLW0_UCSSEL_2; // horloge SPI : SMCLK
    EUSCI_B0->BRW = 0x03; // fréquence de SMCLK (3 Mhz) divisée par 3 (donc 1 Mhz)
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST; // Mettre EUSCI_BO en état actif
}


void init_broches(void)
{
    // configuration des boutons P1.1 et P1.4
    P1->REN |= (BIT1|BIT4);
    P1->OUT |= (BIT1|BIT4);
    P1->IE |= (BIT1|BIT4);
    P1->IES |= (BIT1|BIT4);
    P1->IFG = 0;

    P6->DIR &= ~(BIT0|BIT1|BIT2);
    P6->REN |= (BIT0|BIT1|BIT2);
    P6->OUT |= (BIT0|BIT1|BIT2);
    P6->IE |= (BIT0|BIT1|BIT2);
    P6->IES |= (BIT0|BIT1|BIT2);
    P6->IFG = 0;

    // configuration des LED témoins
    P1->DIR |= BIT0;
    P1->OUT &= ~BIT0;

    P2->DIR |= (BIT0|BIT1|BIT2);
    P2->OUT &= ~(BIT0|BIT1|BIT2);

    // configuration des ports de sortie SPI
    P1->SEL0 |= BIT5 | BIT6 | BIT7; // P1.5, P1.6, P1.7 réglés comme CLK, MOSI et MISO
}


void init_clocks(void)
{
    //------------Configuration de TA0-------------
    TIMER_A0->CTL = TASSEL__ACLK|MC__UP|ID__4|TACLR|TAIE|TAIFG;         // TIMER_A0 sur 16/2 = 4 kHz
    TIMER_A0->CCR[0] = 250-1;          // On définit une fréquence de 16Hz
}


void envoyer_SPI(uint8_t* command,  uint8_t len, uint8_t* res, uint8_t fin)
{
    uint8_t cmpt = 0;
    bool drapeau = 0;
    uint8_t tamp;
    i = 0;                                              // Indice du tableau commande
    do
    {
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)); // on attend que le buffer d'émission soit prêt
        if (i<=len-1)
        {
            EUSCI_B0->TXBUF = command[i];                  // on envoie la donnée suivante (efface TXIFG)
        }
        else
        {
            EUSCI_B0->TXBUF = 0x00;                 // envoie de valeur nul le temps de la réception des données
        }

        tamp = EUSCI_B0->RXBUF;

        if (tamp==0xaf)                         // le premier Byte correspondant à la réponse a bien été reçu
        {
            drapeau = 1;                        // on autorise l'écriture dans res
            cmpt = 0;                           // on remet le compteur à 0     
        }

        if (drapeau==1)
        {
            res[cmpt] = tamp;
        }
        cmpt = cmpt + 1;                        // indice du tableau res
        i = i + 1;                              // indice du tableau commande
        
    } while (cmpt!=fin);                        // tant que le dernier élément de res n'a pas été reçu
}


void LED_on(void)
{
    const int len = 6;
    const int fin = 10;
    uint8_t command[len] = {0xae,0xc1,0x16,0x02,0x01,0x01};
    uint8_t res[fin];
    envoyer_SPI(command,len,res,fin);
}


void LED_off(void)
{
    const int len = 6;
    const int fin = 10;
    uint8_t command[len] = {0xae,0xc1,0x16,0x02,0x00,0x00};
    uint8_t res[fin];
    envoyer_SPI(command,len,res,fin);
}


int getFPS(void)
{
    const int len = 4;
    const int fin = 10;
    uint8_t command[len] = {0xae,0xc1,0x18,0x00};
    uint8_t res[fin];
    envoyer_SPI(command,len,res,fin);

    // Obtenient du nombre de FPS et convertion 0x -> 0d
    uint32_t FPS = 0;
    int max = 3;
    for (j=0; j<=max; j++)
    {
        FPS |= res[9-j];
        if (j!=max)
            FPS <<= 8;
    }
    printf("Nb FPS = %d\n",FPS);
    return FPS;
}


void getBlocks(void)
{
    const int len = 6;
    const int fin = 20;
    uint8_t command[len] = {0xae,0xc1,0x20,0x02,0x07,0x02};
    uint8_t res[fin];
    envoyer_SPI(command,len,res,fin);

    // printf("Res : \n");
    // printf("\t%x%x\n",res[1],res[0]);
    // for (j=2; j<=3; j++)
    // {
    //     printf("\t%x\n",res[j]);
    // }
    // for (j=4; j<=16; j+=2)
    // {
    //     printf("\t%x%x\n",res[j+1],res[j]);
    // }
    // for (j=18; j<=19; j++)
    // {
    //     printf("\t%x\n",res[j]);
    // }
    
    // printf("Signature = \t%x\n",res[6]);
    // printf("\n");

    if (((res[6]==1) || (res[6]==2) || (res[6]==3)) && res[4]!=0)
    {
        front = true;
    }
    else
    {
        front = false;
    }
}


void PORT1_IRQHandler(void)
{
    // Exception sur le bouton P1.1
    if (P1->IFG & BIT1)
    {
        P1->IFG &= ~BIT1;          // On efface le drapeau d'interruption
        if (P1->IES & BIT1)         // allumage des LED boutons appuyé
            LED_on();
        if ((P1->IES & BIT1)==0)    // extinction des LED boutons relaché
            LED_off();
        P1->IES ^= BIT1;
    }


    // Exception sur le bouton P1.4
    if (P1->IFG & BIT4)
    {
        P1->IFG &= ~BIT4;          // On efface le drapeau d'interruption
        P2->OUT |= (BIT0|BIT1|BIT2);
        camera = 1;
        P2->OUT &= ~(BIT0|BIT1|BIT2);
    }
}


void TA0_N_IRQHandler(void)
{
    if (TIMER_A0->CTL & TAIFG)
    {
        TIMER_A0->CTL &= ~TAIFG;        // On efface le drapeau d'interruption

        // P2->OUT ^= (BIT0|BIT1|BIT2);    // clignotement de la LED
        camera = true;
    }
}


int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();
    CS_initClockSignal(CS_MCLK, CS_MODOSC_SELECT, CS_CLOCK_DIVIDER_4);   // horloge MCLK à 6Mhz pour SPI
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_2);   // horloge ACLK à 16 kHz


    //------------Configuration-------------
    init_broches();
    init_SPI();
    init_clocks();


    //------------Activation des interruptions-------------
    NVIC_EnableIRQ(EUSCIB0_IRQn);    // NVIC : Activation des interruptions eUSCI_B0
    NVIC_EnableIRQ(PORT1_IRQn);      // NVIC : Activation des interruptions Port1
    NVIC_EnableIRQ(TA0_N_IRQn);      // NVIC : Activation des interruptions TA0
    NVIC_EnableIRQ(PORT6_IRQn);      // NVIC : Activation des interruptions Port6
    __enable_irq();                  // Activation des interruptions au niveau CPU

    while (1) // boucle infinie
    {
        if (camera==true)           // CF interruption TA0
        {
            getBlocks();            // on vérifie si un block de la liste se trouve devant
            camera = 0;
        }


        if (front==true)            // CF getBlocks
        {
            P2->OUT |= (BIT0|BIT1|BIT2);        // allumage de la LED si dectection d'objet dans la liste enregistré
        }
        else
        {
            P2->OUT &= ~(BIT0|BIT1|BIT2);       // extinction de la LED si aucun objet de la liste enregistré n'est détecté
        }
    }
}
