
// Il faut relier GND, P1.5(UCB0CLK), P1.6(UCBSIMO) et P1.7(UCBSOMI) des 2 microcontr�leurs.
// l'un ex�cute le programme maitre, l'autre le programme esclave
// A chaque appui sur le bouton S1 du ma�tre, une chaine de caract�re est
// envoy�e par le maitre vers l'esclave qui l'affiche alors.

#include <stdint.h>
#include <stdio.h>
#include </Applications/ti/simplelink_msp432p4_sdk_3_40_01_02/source/ti/devices/msp432p4xx/driverlib/driverlib.h>

int len = 5;
uint8_t getBlocks[] = {0xae,0xc1,22,2,1,1};
int i;


int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();


    P1->SEL0 = P1->SEL1 = P2->SEL0 = P2->SEL1 = 0;

    // configuration du bouton sur P1.1
    P1->DIR &= ~BIT1;
    P1->REN |= BIT1;
    P1->OUT |= BIT1;


    // configuration de la LED sur P1.0
    P1->DIR |= BIT0;  // P1.0 sortie
    P1->OUT &= ~BIT0; // P1.0 �teinte

    //r�glage des LEDs P2
    P2->DIR |= BIT0;  // P2.0 sortie
    P2->OUT &= ~BIT0; // P2.0 �teinte
    P2->DIR |= BIT1;  // P2.1 sortie
    P2->OUT &= ~BIT1; // P2.1 �teinte
    P2->DIR |= BIT2;  // P2.2 sortie
    P2->OUT &= ~BIT2; // P2.2 �teinte



    P6->SEL0 |= BIT2|BIT3|BIT4|BIT5; // P6.2, P6.3, P6.4, P6.5 r�gl�s comme UCB1STE, UCB0CLK, UCBSIMO et UCBSOMI
    P6->SEL1 |= BIT2|BIT3|BIT4|BIT5; // P6.2, P6.3, P6.4, P6.5 r�gl�s comme UCB1STE, UCB0CLK, UCBSIMO et UCBSOMI


    //------Configuration du mode SPI ma�tre----------------------------------------------------
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Mettre EUSCI_BO en �tat reset
    EUSCI_B0->CTLW0 |=  EUSCI_B_CTLW0_MST|  // Mode ma�tre
                        EUSCI_B_CTLW0_SYNC| // Mode synchrone (SPI)
                        EUSCI_B_CTLW0_CKPL| // Polarit�, �tat inactif � 1
                        EUSCI_B_CTLW0_MSB|  // Envoi du MSB en premier
                        EUSCI_B_CTLW0_UCSSEL_2; // horloge SPI : SMCLK
    EUSCI_B0->BRW = 0x03; // fr�quence de SMCLK (3 Mhz) divis�e par 3 (donc 1 Mhz)
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST; // Mettre EUSCI_BO en �tat actif
    //------------------------------------------------------------------------------------------

    NVIC_EnableIRQ(PORT1_IRQn);      // NVIC : Activation des interruptions Port1
    NVIC_EnableIRQ(EUSCIB0_IRQn);    // NVIC : Activation des interruptions eUSCI_B0
    __enable_irq();                  // Activation des interruptions au niveau CPU


    // Interruptions pour le bouton P1.1
    P1->IES |= BIT1;     // interruption sur front descendant sur P1.1
    P1->IE = BIT1;       // autorisation des interruptions UNIQUEMENT SUR BIT1
    P1->IFG = 0;         // effacement du drapeau d'interruption


    while(1); // boucle infinie
}

void PORT1_IRQHandler (void)
{
    if (P1->IFG & BIT1)
    {
        P1->IFG &= ~BIT1; // on efface le drapeau d'interruption de P1.1

        
        i=1;
        do
        {
            while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)); // on attend que le buffer d'�mission soit pr�t

                    // on envoie la donn�e suivante
                    EUSCI_B0->TXBUF = getBlocks[i];

                    if (i==1||i==4)
                    {
                        P2->OUT |= BIT0;
                        _delay_cycles(300000);
                        P2->OUT &= ~(BIT0|BIT1|BIT2);
                    }
                    else if (i==2||i==5)
                    {
                        P2->OUT |= BIT1;
                        _delay_cycles(300000);
                        P2->OUT &= ~(BIT0|BIT1|BIT2);
                    }
                    else if (i==3||i==6)
                    {
                        P2->OUT |= BIT2;
                        _delay_cycles(300000);
                        P2->OUT &= ~(BIT0|BIT1|BIT2);
                    }

                    i+=1;
        }

        while (i!=len);
        P1->OUT |= BIT0;
        _delay_cycles(450000);
        P1->OUT &= ~BIT0;

    }
}

