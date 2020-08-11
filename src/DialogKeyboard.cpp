#include "DialogKeyboard.h"
#include <QtWidgets>
#include "ui_DialogKeyboard.h"

DialogKeyboard::DialogKeyboard(QWidget *parent) : QDialog(parent), ui(new Ui::DialogKeyboard)
{
    ui->setupUi(this);
}

DialogKeyboard::~DialogKeyboard()
{
    delete ui;
}

void DialogKeyboard::keyPressEvent(QKeyEvent *event)
{
    if (!drone)
        return;
    char key = event->key();
    std::cout << "key:" << key << std::endl;

    switch (key)
    {
    case 'X': //land
        drone->land();
        break;
    case 'I': //going up
        drone->rise(0.4f);
        break;
    case 'K': //going down
        drone->rise(-0.4f);
        break;
    case 'J': //turn left
        drone->yaw(0.4f);
        break;
    case 'L': //turn right
        drone->yaw(-0.4f);
        break;
    case 'A': //tilt left
        if (!drone->isVelMode)
            drone->roll(-0.1f);
        else
            drone->pitch(0.7f);

        break;
    case 'D': //tilt right
        if (!drone->isVelMode)
            drone->roll(0.1f);
        else
            drone->pitch(-0.7f);
        break;
    case 'W': //title front
        if (!drone->isVelMode)
            drone->pitch(0.1f);
        else
            drone->roll(0.7f);
        break;
    case 'S':
        if (!drone->isVelMode)
            drone->pitch(-0.1f);
        else
            drone->roll(-0.7f);
        break;
    case 'T':
        testPositionControl();
        break;
    }
    event->accept();
}

void DialogKeyboard::keyReleaseEvent(QKeyEvent *event)
{
    if (!drone)
        return;
    char key = event->key();
    if (!event->isAutoRepeat()) {
        std::cout << "key:" << key << " has been released !" << std::endl;
        event->accept();
    }
    else {
        event->ignore();
    }
}

void DialogKeyboard::testPositionControl()
{
    while (1) {
        drone->moveTo(3, 3, 3);
        sleep(1);
        drone->moveTo(-3, -3, -3);
        sleep(1);
        drone->moveTo(-3, 3, 3);
        sleep(1);
        drone->moveTo(3, -3, -3);
        sleep(1);
        // drone->moveTo(-3, 3, 3);
        // sleep(1);
        // // drone->moveTo(0, 0, 0);
        // sleep(1);
        // drone->moveTo(3, -3, -3);
        // sleep(1);
        // drone->moveTo(0, 0, 0   );
        // sleep(1);
        // drone->moveTo(1, 1, 1);
        // sleep(5);
        // drone->moveTo(1, 1, 1);
    }
}
