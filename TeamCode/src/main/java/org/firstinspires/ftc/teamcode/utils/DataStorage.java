package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;


public class DataStorage {
    public static Pose actualPose = null;
    public static AllianceEnum alliance = AllianceEnum.Red;
    public static int pieceCount = 0;
    public static boolean DEBUG_MODE = false;

    /**
     * Qual CÉLULA do HIVE da aliança ativa está voltada pra cima agora (portanto pontuável).
     * true  = célula do LADO DA PLATEIA está pra cima.
     * false = célula do LADO OPOSTO À PLATEIA está pra cima.
     *
     * <p>Default (true) corresponde ao estado de staging oficial do RED (Section 10.3.1 do
     * manual). Sempre que a aliança for definida/confirmada (Autos.prepareRoutine), chame
     * {@link HiveTargets#resetForMatchStart(AllianceEnum)} pra corrigir esse valor pro BLUE
     * também, já que o staging inicial NÃO é o mesmo lado pras duas alianças.
     */
    public static boolean activeCellIsAudienceSide = true;

}
