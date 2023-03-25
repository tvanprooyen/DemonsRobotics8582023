package frc.robot.Util;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class MatchData {

    private int getGameMode;
    private String driverSelect;

    public enum Actions {
        LOWGOAL,
        MIDGOAL,
        HIGHGOAL,
        STORE,
        RELEASE,
        INSIDE,
        MODE
    }

    public MatchData() {
        this.getGameMode = 0;
        this.driverSelect = "Gabe";
    }

    public void setDriverSelect(String driverSelect) {
        this.driverSelect = driverSelect;
    }

    public int getProfileButton(Actions actions) {
        int selectedButton;

        if(this.driverSelect == "Gabe") {
            switch (actions) {
                case LOWGOAL: selectedButton = 3; break;
                case MIDGOAL: selectedButton = 5; break;
                case HIGHGOAL: selectedButton = 6; break;
                case STORE: selectedButton = 1; break;
                case RELEASE: selectedButton = 4; break;
                case INSIDE: selectedButton = 8; break;
                case MODE: selectedButton = 2;
                default: selectedButton = 5; break;
            }
        } else {
            switch (actions) {
                case LOWGOAL: selectedButton = 1; break;
                case MIDGOAL: selectedButton = 3; break;
                case HIGHGOAL: selectedButton = 4; break;
                case STORE: selectedButton = 5; break;
                case RELEASE: selectedButton = 6; break;
                case INSIDE: selectedButton = 8; break;
                case MODE: selectedButton = 2;
                default: selectedButton = 2; break;
            }
        }
        

        return selectedButton;
    }

    public double getProfileSlewRate() {
        if(this.driverSelect == "Gabe") {
            return 15;
        } else {
            return 5;
        }
    }

    public String getDriverSelect() {
        return this.driverSelect;
    }

    public void setGetGameMode(int getGameMode) {
        this.getGameMode = getGameMode;
    }


    public int getGetGameMode() {
        return this.getGameMode;
    }
    
}
