#include "ti/devices/msp432p4xx/inc/msp.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// |-----------------------------------Déclaration et réglage des paramètres-----------------------------------|
int mode = 1; // mode select, 1 = mode auto, 2 = mode joystik

// Parametres pour SPI
int i = 0;
int j = 0;
bool camera = false;
bool front = false;
bool is_tapis_mooving = false;

// Parametres pour joysticks
int is_mooving_0 = 0;
int is_mooving_1 = 0;
int is_mooving_2 = 0;
int is_mooving_3 = 0;
uint16_t resultTab[4];
int moteur_0;
int moteur_1;
int moteur_2;
int moteur_3;

// Parametres pour UART
char chaine[];  // Chaîne de commandes à envoyer

// |-----------------------------------Initialisations-----------------------------------|
void init_SPI(void)
{
    //------Configuration du mode SPI maître----------------------------------------------------
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Mettre EUSCI_BO en état reset
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MST |  // Mode maître
        EUSCI_B_CTLW0_SYNC | // Mode synchrone (SPI)
        EUSCI_B_CTLW0_CKPL | // Polarité, état inactif à 1
        EUSCI_B_CTLW0_MSB |  // Envoi du MSB en premier
        EUSCI_B_CTLW0_UCSSEL_2; // horloge SPI : SMCLK
    EUSCI_B0->BRW = 0x03; // fréquence de MCLK (6 Mhz) divisée par 3 (donc 3 Mhz)
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST; // Mettre EUSCI_BO en état actif
}

void InitUart(void){

    //------------Initialisation UART--------------------------------------------------
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST;  // reset
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK; // clock source: SMCLK 12MHz
    EUSCI_A2->BRW = 6;                            // baud rate: 115200 bps: 12MHz / 115200 = 104.1667
    EUSCI_A2->MCTLW = (8<<EUSCI_A_MCTLW_BRF_OFS)     // config. Baudrate : BRF=8
                      |(0x20<<EUSCI_A_MCTLW_BRS_OFS) // config. Baudrate : BRS=0x20
                      |EUSCI_A_MCTLW_OS16;           // config. Baudrate : OS16=1
    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // clear reset
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE; // Autorisation des interruptions pour la réception de donnés
}

void init_broches(void)
{
    // configuration des boutons P1.1 et P1.4
    P1->REN |= (BIT1|BIT4);   // résistance de PULL sur P1.1 et P1.4
    P1->OUT |= (BIT1|BIT4);   // résistance en PULL-UP sur P1.1 et P1.4
    P1->IE |= (BIT1|BIT4);    // autorisation des interruptions sur P1.1 et P1.4 
    P1->IES |= (BIT1|BIT4);   // interruptions sur front descendant
    P1->IFG &= ~(BIT1|BIT4);  // remise à zéro des drapeaux d'interruption P1.1 et P1.4

    // configuration du premier joystik
    P4->DIR  &= ~(BIT0|BIT2|BIT4); // P4.0 P4.2 P4.4 en entrée

    P4->SEL0 |= (BIT2|BIT0);  // Fonction ADC pour P4.0 P4.2
    P4->SEL1 |= (BIT2|BIT0);  // Fonction ADC pour P4.0 P4.2

    P4->SEL1 = P4->SEL0 &= ~BIT4;  // Fonction GPIO pour P4.4
    P4->REN |=  BIT4;   // résistance de PULL sur P4.4
    P4->OUT &=  ~BIT4;  // résistance en PULL-UP sur P4.4
    P4->IE  |=  BIT4;   // autorisation des interruptions sur P4.4
    P4->IES |=  BIT4;   // interruptions sur front descendant
    P4->IFG &= ~BIT4;   // remise à zéro du drapeau d'interruption P4.4

    // configuration du dexuième joystik
    P5->DIR  &= ~(BIT0|BIT1|BIT2); // P5.0 P5.1 P5.2 en entrée

    P5->SEL0 |= (BIT1|BIT0);  // Fonction ADC pour P5.0 P5.1
    P5->SEL1 |= (BIT1|BIT0);  // Fonction ADC pour P5.0 P4.1

    P5->SEL1 = P5->SEL0 &= ~BIT2;  // Fonction GPIO pour P5.2
    P5->REN |=  BIT2;   // résistance de PULL sur P5.2
    P5->OUT &=  ~BIT2;  // résistance en PULL-UP sur P5.2
    P5->IE  |=  BIT2;   // autorisation des interruptions sur P5.2
    P5->IES |=  BIT2;   // interruptions sur front descendant
    P5->IFG &= ~BIT2;   // remise à zéro du drapeau d'interruption P5.2

    // configuration des LED témoins
    P1->DIR |= BIT0;
    P1->OUT &= ~BIT0;

    P2->DIR |= (BIT0|BIT1|BIT2);
    P2->OUT &= ~(BIT0|BIT1|BIT2);

    // configuration des ports de sortie SPI
    P1->SEL0 |= BIT5 | BIT6 | BIT7; // P1.5, P1.6, P1.7 réglés comme CLK, MOSI et MISO

    // configuration des ports de UART
    P3->SEL0 |= (BIT2 | BIT3);   // MSP: P3.2>>RX,P3.3>>TX;  LSS: 0>>TX,1>>RX.   P3.3-->1, 0<--P3.2
    P3->SEL1 &= ~(BIT2 | BIT3);  // SEL 01 UART
}

void init_clocks(void)
{
    //------------Configuration de TA0 pour la caméra-------------
    TIMER_A0->CTL = TASSEL__ACLK|MC__UP|ID__4|TACLR|TAIE|TAIFG;         // TIMER_A0 sur 16/2 = 4 kHz
    TIMER_A0->CCR[0] = 250-1;          // On définit une fréquence de 16Hz

    //------------Configuration de TA1 pour le module ADC-------------
    TIMER_A1->CTL = TASSEL__ACLK|MC__UP|TACLR;
    TIMER_A1->CCR[0] = 32000-1;      // TA1.0 compte 1 s (32000/32khz)
    TIMER_A1->CCR[1] = 16000;        // TA1.1 compte 0.5 s (16000/32khz)
    TIMER_A1->CCTL[1] = OUTMOD_3 ;   // Réglage par défaut en mode compare (CAP=0)
}

void init_ADC(void)
{
    // Initialisation du module ADC
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

    // Configuration des mémoires ADC : ADC_MEM0 - ADC_MEM3 (A13, A11, A5, A4)
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM3, true);
    ADC14_configureConversionMemory(ADC_MEM0,ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);
    ADC14_configureConversionMemory(ADC_MEM1,ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS);
    ADC14_configureConversionMemory(ADC_MEM2,ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A5, ADC_NONDIFFERENTIAL_INPUTS);
    ADC14_configureConversionMemory(ADC_MEM3,ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A4, ADC_NONDIFFERENTIAL_INPUTS);


    // Configuration de la source de déclenchement
    ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE3, false);

    // type d'échantillonage (manuel ou automatique)
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    // autorisation des interruptions sur ADC_MEM3 (dernier de la liste des ADC_MEM)
    ADC14_enableInterrupt(ADC_INT3);
}



// |------------------------------------Protocoles d'envoi---------------------------------------|
void SendUART(char chaine[])
{
    char *caractere = chaine; // pointer set to the 1st character of the chain
    while (*caractere != 0) // tant qu'on n'est pas en fin de chaine
    {
        while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG)); // attente de pouvoir envoyer un caractère
        EUSCI_A2->TXBUF = *caractere;
        caractere++;                                  // on passe au caractère suivant
    }
}

void envoyer_SPI(uint8_t* command,  uint8_t len, uint8_t* res, uint8_t fin)
{
    uint8_t cmpt = 0;                                   // indice du tableau res
    bool drapeau = 0;                                   // drapeau de réception
    uint8_t tamp;
    i = 0;                                              // indice du tableau commande
    do
    {
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG));   // on attend que le buffer d'émission soit prêt
        if (i<=len-1)
        {
            EUSCI_B0->TXBUF = command[i];                  // on envoie la donnée suivante (efface TXIFG)
        }
        else
        {
            EUSCI_B0->TXBUF = 0x00;                 // envoie de valeur nul le temps de recevoir les données si la commande à fini d'être envoyé
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



// |------------------------------Fonctions--------------------------------|
// --------------SPI--------------
void LED_on(void)
{
    const int len = 6;      // longueur de la commande
    const int fin = 10;     // longeur de la réponse attendue
    uint8_t command[len] = {0xae,0xc1,0x16,0x02,0x01,0x01};
    uint8_t res[fin];
    envoyer_SPI(command,len,res,fin);
}

void LED_off(void)
{
    const int len = 6;      // longueur de la commande
    const int fin = 10;     // longeur de la réponse attendue
    uint8_t command[len] = {0xae,0xc1,0x16,0x02,0x00,0x00};
    uint8_t res[fin];
    envoyer_SPI(command,len,res,fin);
}

void getBlocks(void)
// Permet d'obtenir la signature des éléments détectés
{
    const int len = 6;      // longueur de la commande
    const int fin = 20;     // longeur de la réponse attendue
    uint8_t command[len] = {0xae,0xc1,0x20,0x02,0x07,0x02};
    uint8_t res[fin];
    envoyer_SPI(command,len,res,fin);

    // res[6] = signature de l'objet
    // res[4] = nombre d'objet présent dans le champ de vision de la caméra
    if (( (res[6]==1) || (res[6]==2) || (res[6]==3) || (res[6]==4)) && res[4]!=0)
    {
        front = true;           // un objet répertorien se trouve devant la caméra
    }
    else
    {
        front = false;          // il n'y a aucun objet en mémoire devant la caméra
    }
}


// --------------UART--------------
void LED(void)             // Tous les LEDs du moteur sont allumés en bleu
{
    strcpy(chaine, "#0LED3\r");
    SendUART(chaine);
    strcpy(chaine, "#1LED3\r");
    SendUART(chaine);
    strcpy(chaine, "#2LED3\r");
    SendUART(chaine);
    strcpy(chaine, "#3LED3\r");
    SendUART(chaine);
    strcpy(chaine, "#4LED3\r");
    SendUART(chaine);
    strcpy(chaine, "#5LED3\r");
    SendUART(chaine);
}

void tapis_on(void)       // tapis commence à se déplacer
{
    strcpy(chaine, "#5WD150\r");
    SendUART(chaine);
}

void tapis_off(void)      // tapis s'arrête
{
    strcpy(chaine, "#5H\r");
    SendUART(chaine);
}

void move2Init(void)     // déplacement vers la position initiale et attente du début du mouvement
{
    strcpy(chaine, "#0D450T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#1D-600T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#2D300T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#3D1200T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#4D900T2000\r");
    SendUART(chaine);
}

char move2Grab(void)     // mouvement vers la position au-dessus de l'objet
{
    strcpy(chaine, "#0D211T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#1D270T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#2D-50T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#3D657T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#4D0T2000\r");
    SendUART(chaine);
}

void moveGrab(void)      // la pince prend l'objet
{
    strcpy(chaine, "#4D800T500\r");
    SendUART(chaine);
}

void move2MiPoint(void)    // position du point médian de la trajectoire qui soulève le bras du robot pour éviter les collisions
{
    strcpy(chaine, "#0D-300T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#1D000T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#2D-300T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#3D800T2000\r");
    SendUART(chaine);
}

void move2Drop(void)  // mouvement vers la position de placement
{
    strcpy(chaine, "#0D-900T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#1D500T2000\r");
    SendUART(chaine);
    strcpy(chaine, "#2D-300T2000\r");
    SendUART(chaine);
}

void moveDrop(void)   // la pince relâche et dépose l'objet
{
    strcpy(chaine, "#4D500T500\r");
    SendUART(chaine);
}

void scenario(void)   // L'ensemble du processus de mouvement
{
    move2Init();
    __delay_cycles(20000000);  // lelai pour moteur tourner
    move2Grab();
    __delay_cycles(20000000);
    moveGrab();
    __delay_cycles(20000000);
    move2MiPoint();
    __delay_cycles(20000000);
    move2Drop();
    __delay_cycles(20000000);
    moveDrop();
    __delay_cycles(10000000);
    move2Init();
}

void moteur_on_T(uint8_t i)    // Ensemble d'instructions pour les moteurs en sens positif
{
    char* command;

    switch (i)
    {
    case 0:
        command = "#0WD1800\r";  // moteur 0 est en sens positif à 180°/s
        break;
    case 1:
        command = "#1WD1800\r";  // moteur 1 est en sens positif à 180°/s
        break;
    case 2:
        command = "#2WD1800\r";  // moteur 2 est en sens positif à 180°/s
        break;
    case 3:
        command = "#3WD1800\r";  // moteur 3 est en sens positif à 180°/s
        break;
    default:
        break;
    }
    SendUART(command);
}

void moteur_on_H(uint8_t i)   // Ensemble d'instructions pour les moteurs en sens négatif
{
    char* command;

    switch (i)
    {
    case 0:
        command = "#0WD-1800\r";  // moteur 0 est en sens négatif à 180°/s
        break;
    case 1:
        command = "#1WD-1800\r";  // moteur 1 est en sens négatif à 180°/s
        break;
    case 2:
        command = "#2WD-1800\r";  // moteur 2 est en sens négatif à 180°/s
        break;
    case 3:
        command = "#3WD-1800\r";  // moteur 3 est en sens négatif à 180°/s
        break;
    default:
        break;
    }
    SendUART(command);
}

void moteur_off(uint8_t i)  // Ensemble d'instructions pour les moteurs stop
{
    char* command;

    switch (i)
    {
    case 0:
        command = "#0H\r";  // moteur 0 s'arrête
        break;
    case 1:
        command = "#1H\r";  // moteur 0 s'arrête
        break;
    case 2:
        command = "#2H\r";  // moteur 0 s'arrête
        break;
    case 3:
        command = "#3H\r";  // moteur 0 s'arrête
        break;
    case 4:
        command = "#4H\r";  // moteur 0 s'arrête
        break;
    default:
        break;
    }
    SendUART(command);
}



// |--------------------Interruptions-----------------------|
void PORT1_IRQHandler(void)
{
    // Exception sur le bouton P1.1 : passage en mode auto
    if (P1->IFG & BIT1)
    {
        P1->IFG &= ~BIT1;          // On efface le drapeau d'interruption
        mode = 1;                  // mode auto
        tapis_on();                // mise en marche du tapis

        // Mise à jour des autorisations d'interruption
        TIMER_A0->CTL |= TAIE;            // Activation des interruptions TA0
        P4->IE &= ~BIT4;                  // NVIC : Désactivation des interruptions Port4
        P5->IE &= ~BIT2;                  // NVIC : Désactivation des interruptions Port5
        NVIC_DisableIRQ(ADC14_IRQn);      // NVIC : Désactivation des interruptions ADC
        ADC14_disableConversion();        // Désactivation des conversions ADC
        
        // clignotement de la LED (un seul) pour indiquer le bon passage en mode auto
        P1->OUT |= BIT0;            
        __delay_cycles(15000);
        P1->OUT &= ~BIT0;
    }


    // Exception sur le bouton P1.4 : passage en mode manuel
    if (P1->IFG & BIT4)
    {
        P1->IFG &= ~BIT4;          // On efface le drapeau d'interruption
        mode = 2;                  // mode manuel
        tapis_off();               // arrêt du tapis

        // Mise à jour des autorisations d'interruption
        TIMER_A0->CTL &= ~TAIE;             // Désactivation des interruptions TA0
        P4->IE |= BIT4;                     // NVIC : Activation des interruptions Port4
        P5->IE |= BIT2;                     // NVIC : Activation des interruptions Port5
        NVIC_SetPriority(ADC14_IRQn,6);     // NVIC : Priorité minimal pour l'ADC
        NVIC_EnableIRQ(ADC14_IRQn);         // NVIC : Activation des interruptions ADC
        ADC14_enableConversion();           // Activation des conversions ADC
        ADC14_toggleConversionTrigger();    // Lancement de la première conversion

        // clignotement de la LED (deux successifs) pour indiquer le bon passage en mode manuel
        P1->OUT |= BIT0;
        __delay_cycles(15000);
        P1->OUT &= ~BIT0;

        __delay_cycles(1500000);

        P1->OUT |= BIT0;
        __delay_cycles(15000);
        P1->OUT &= ~BIT0;
    }
}

void PORT4_IRQHandler(void)
{
    // Exception sur P4.4 : commande de fermeture de la pince
    if (P4->IFG & BIT4)
    {
        __delay_cycles(45000);     // Delais pour limiter les effets rebond
        P4->IFG &= ~BIT4;          // On efface le drapeau d'interruption
        moveGrab();                // La pince se ferme
    }
}

void PORT5_IRQHandler(void)
// Commande d'ouverture de la pince, déclenchée par P5.2
{
    // Exception sur P5.2
    if (P5->IFG & BIT2)
    {
        __delay_cycles(45000);     // Delais pour limiter les effets rebond
        P5->IFG &= ~BIT2;          // On efface le drapeau d'interruption
        moveDrop();                // La pince s'ouvre
    }
}

void TA0_N_IRQHandler(void)
{
    if (TIMER_A0->CTL & TAIFG)
    {
        TIMER_A0->CTL &= ~TAIFG;        // On efface le drapeau d'interruption
        camera = true;                  // Signifie que la fonction getBlocks va être appeler dans la boucle while
    }
}

void ADC14_IRQHandler(void)
{
    uint64_t status;
    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);


    if(status & ADC_INT3)
    {
        resultTab[0] = ADC14_getResult(ADC_MEM0);       // numérisation de l'axe vertical joystick droit
        resultTab[1] = ADC14_getResult(ADC_MEM1);       // numérisation de l'axe horizontal joystick droit
        resultTab[2] = ADC14_getResult(ADC_MEM2);       // numérisation de l'axe vertical joystick gauche
        resultTab[3] = ADC14_getResult(ADC_MEM3);       // numérisation de l'axe horizontal joystick droit

        // moteur = 0 : stop
        // moteur = 1 : rotation sens trigo
        // moteur = 2 : rotation sens horaire

        // axe horozontal joystick droit contrôle le moteur 2
        if ((resultTab[0]>200) && (resultTab[0]<14000))     // joystick au centre
            moteur_2 = 0;
        if (resultTab[0]>14000)                             // joystick en haut
            moteur_2 = 2;
        if (resultTab[0]<200)
            moteur_2 = 1;

        // axe vertical joystick droit contrôle le moteur 1
        if ((resultTab[1]>200) && (resultTab[1]<14000))     // joystick au centre
            moteur_1 = 0;
        if (resultTab[1]>14000)
            moteur_1 = 1;
        if (resultTab[1]<200)
            moteur_1 = 2;

        // axe vertical joystick gauche coontrôle le moteur 3
        if ((resultTab[2]>200) && (resultTab[2]<14000))     // joystick au centre
            moteur_3 = 0;
        if (resultTab[2]>14000)                             // joystick en bas
            moteur_3 = 2;
        if (resultTab[2]<200)
            moteur_3 = 1;

        // axe horizontal joystick gauche contrôle le moteur 0
        if ((resultTab[3]>200) && (resultTab[3]<14000))     // joystick au centre
            moteur_0 = 0;
        if (resultTab[3]>14000)
            moteur_0 = 2;
        if (resultTab[3]<200)
            moteur_0 = 1;
    }
}


int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();
    CS_initClockSignal(CS_MCLK, CS_MODOSC_SELECT, CS_CLOCK_DIVIDER_4);   // horloge MCLK à 6Mhz pour SPI
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);                     // horloge SMCLK À 12Mhz pour UART
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_2);   // horloge ACLK à 16 kHz

    //------------Configuration------------
    init_broches();
    init_SPI();
    InitUart();
    init_clocks();
    init_ADC();

    //------------Activation des interruptions------------
    NVIC_EnableIRQ(EUSCIB0_IRQn);    // NVIC : Activation des interruptions eUSCI_B0
    NVIC_EnableIRQ(PORT1_IRQn);      // NVIC : Activation des interruptions Port1
    NVIC_EnableIRQ(PORT4_IRQn);      // NVIC : Activation des interruptions Port4
    NVIC_EnableIRQ(PORT5_IRQn);      // NVIC : Activation des interruptions Port5
    NVIC_EnableIRQ(TA0_N_IRQn);      // NVIC : Activation des interruptions TA0
    NVIC_SetPriority(ADC14_IRQn,6);  // NVIC : Priorité minimal pour l'ADC
    NVIC_EnableIRQ(ADC14_IRQn);      // NVIC : Activation des interruptions ADC
    __enable_irq();                  // Activation des interruptions au niveau CPU
    P1->IFG |= BIT1;                 // Activation logiciel du mode auto

    //------------initialisation du bras et du tapis------------
    LED();              // LEDs des motors allument
    move2Init();        // move to initial position
    tapis_on();         // mise en marche du tapis

    while (1)
    {
        //---------------Routine Automatique----------------
        if (mode == 1)  // mode auto
        {
            if (camera==true)           // CF interruption TA0
                {
                    getBlocks();            // on vérifie si un block de la liste se trouve devant
                    camera = 0;
                }
                if (front==true)            // CF getBlocks
                {
                    P2->OUT |= (BIT0|BIT1|BIT2);        // allumage de la LED si dectection d'objet dans la liste enregistrée
                    tapis_off();                        // tapis s'arrête
                    is_tapis_mooving = false;
                    scenario();                         // Exécuter le scénario
                }
                if ((front==false) && (is_tapis_mooving==false))
                {
                    P2->OUT &= ~(BIT0|BIT1|BIT2);       // extinction de la LED si aucun objet de la liste enregistrée n'est détecté
                    tapis_on();                         // mise en marche du tapis
                    is_tapis_mooving = true;
                }
        }

        //----------------Routine joystick-------------------
        if (mode == 2)  // mode manuel
        {
            if ((moteur_0==0) && (is_mooving_0!=0))
            {
                moteur_off(0);      // arrêt du moteur 0
                is_mooving_0=0;     // témoin d'arrêt moteur 0
            }
            if ((moteur_0==1) && (is_mooving_0!=1))
            {
                moteur_on_T(0);     // rotation dans le sens trigo du moteur 0
                is_mooving_0=1;     // témoin rotation trigo moteur 0
            }
            if ((moteur_0==2) && (is_mooving_0!=2))
            {
                moteur_on_H(0);     // rotation dans le sens horaire du moteur 0
                is_mooving_0=2;     // témoin rotation horaire moteur 0
            }

            if ((moteur_1==0)  && (is_mooving_1!=0))
            {
                moteur_off(1);      // arrêt du moteur 1
                is_mooving_1=0;     // témoin d'arrêt moteur 1
            }
            if ((moteur_1==1)  && (is_mooving_1!=1))
            {
                moteur_on_T(1);     // rotation dans le sens trigo du moteur 1
                is_mooving_1=1;     // témoin rotation trigo moteur 1
            }
            if ((moteur_1==2)  && (is_mooving_1!=2))
            {
                moteur_on_H(1);     // rotation dans le sens horaire du moteur 1
                is_mooving_1=2;     // témoin rotation horaire moteur 1
            }

            if ((moteur_2==0) && (is_mooving_2!=0))
            {
                moteur_off(2);      // arrêt du moteur 2
                is_mooving_2=0;     // témoin d'arrêt moteur 2
            }
            if ((moteur_2==1) && (is_mooving_2!=1))
            {
                moteur_on_T(2);     // rotation dans le sens trigo du moteur 2
                is_mooving_2=1;     // témoin rotation trigo moteur 2
            }
            if ((moteur_2==2) && (is_mooving_2!=2))
            {
                moteur_on_H(2);     // rotation dans le sens horaire du moteur 2
                is_mooving_2=2;     // témoin rotation horaire moteur 2
            }

            if ((moteur_3==0)  && (is_mooving_3!=0))
            {
                moteur_off(3);      // arrêt du moteur 3
                is_mooving_3=0;     // témoin d'arrêt moteur 3
            }
            if ((moteur_3==1)  && (is_mooving_3!=1))
            {
                moteur_on_T(3);     // rotation dans le sens trigo du moteur 3
                is_mooving_3=1;     // témoin rotation trigo moteur 3
            }
            if ((moteur_3==2)  && (is_mooving_3!=2))
            {
                moteur_on_H(3);     // rotation dans le sens horaire du moteur 3
                is_mooving_3=2;     // témoin rotation horaire moteur 3
            }
        }
    }
}
