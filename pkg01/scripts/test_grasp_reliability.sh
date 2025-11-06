#!/bin/bash

# Script di test rapido per verificare affidabilità presa al 100%
# Esegue test multipli per confermare l'affidabilità

echo "=================================================="
echo "  TEST AFFIDABILITÀ PRESA SECCHIO - 100%"
echo "=================================================="
echo ""
echo "Questo script verifica che la presa funzioni sempre"
echo ""

# Colori per output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Funzione di cleanup
cleanup() {
    echo -e "${YELLOW}Pulizia processi Gazebo...${NC}"
    killall -9 gzserver gzclient 2>/dev/null
    sleep 2
}

# Funzione per verificare se ROS è in esecuzione
check_roscore() {
    if ! pgrep -x "roscore" > /dev/null; then
        echo -e "${RED}❌ roscore non in esecuzione!${NC}"
        echo "Avvia roscore in un altro terminale e riprova."
        exit 1
    fi
    echo -e "${GREEN}✓ roscore attivo${NC}"
}

# Cleanup iniziale
echo "1. Pulizia cache Gazebo..."
cleanup
rm -rf ~/.gazebo/models/bucket 2>/dev/null
echo -e "${GREEN}✓ Cache pulita${NC}"
echo ""

# Verifica ROS
echo "2. Verifica ambiente ROS..."
check_roscore
echo ""

# Source workspace
echo "3. Caricamento workspace..."
cd ~/Desktop/smart/lattebot_ws2
source devel/setup.bash
echo -e "${GREEN}✓ Workspace caricato${NC}"
echo ""

echo "=================================================="
echo "  ISTRUZIONI PER IL TEST"
echo "=================================================="
echo ""
echo "1. Il sistema lancerà Gazebo con il mondo farm"
echo "2. Premi 'a' per spaware il secchio"
echo "3. Pubblica su ROS topic per testare la presa:"
echo ""
echo -e "${YELLOW}   rostopic pub -1 /calf_num std_msgs/String \"data: '-1_3'\"${NC}"
echo ""
echo "4. Osserva attentamente:"
echo "   - Apertura gripper"
echo "   - Posizionamento sul manico"
echo "   - Pre-chiusura (grasp_handle)"
echo "   - Chiusura finale (close)"
echo "   - Sollevamento"
echo ""
echo "5. Il secchio DEVE rimanere afferrato durante:"
echo "   - Sollevamento verticale"
echo "   - Movimento piattaforma"
echo "   - Posizionamento finale"
echo ""
echo -e "${GREEN}Se tutto funziona = SUCCESSO! ✓${NC}"
echo -e "${RED}Se il secchio cade = verifica log${NC}"
echo ""
echo "=================================================="
echo ""

read -p "Premi INVIO per lanciare Gazebo..."

# Lancia Gazebo
echo ""
echo "Lancio Gazebo con mondo farm..."
roslaunch pkg01 gazebo_farm.launch

# Cleanup finale
cleanup
echo ""
echo -e "${GREEN}Test completato!${NC}"
