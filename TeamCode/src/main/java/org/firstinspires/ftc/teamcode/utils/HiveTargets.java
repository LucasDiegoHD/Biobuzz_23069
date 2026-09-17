package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * Resolve qual CÉLULA do HIVE está voltada pra cima (portanto pontuável) para a aliança ativa,
 * e fornece a pose de mundo (X, Y) daquela célula pra uso com {@code KinematicAimDriveCommand}
 * e {@code ActiveAimCommand}.
 *
 * <h2>Por que isso existe</h2>
 * Cada HIVE (vermelho e azul) tem 2 CÉLULAS — uma do lado da plateia, outra do lado oposto —
 * mas só a que está voltada pra cima recebe POLLEN/NECTAR. A cada HIVE TIP elas trocam de
 * posição. Detectar o TIP por sensor é caro e pouco confiável por enquanto, então o motorista
 * alterna manualmente com um botão no TeleOp (ver {@code teleop.java}, Left Bumper) sempre que
 * perceber/realizar um TIP.
 *
 * <h2>Coordenadas (medidas pelo time — Team #23069)</h2>
 * X = 60/84 é o centro de cada HIVE (Red/Blue). Y = 52/92 é o ponto de mira escolhido — de
 * propósito NÃO é o centro geométrico da célula (que ficaria em ~60.6/81.4): mira-se mais na
 * frente/boca da célula, mais perto do atirador, pra ter mais margem de erro (curto ainda cai
 * dentro da abertura, em vez de arriscar atravessar e quicar pra fora do fundo, que só tem
 * 12in de profundidade). São {@code @Configurable} pra ajuste fino ao vivo pelo painel Panels.
 *
 * <h2>Staging inicial (Section 10.3.1 / Figura 10-2 do manual)</h2>
 * <ul>
 *   <li><b>RED</b> começa com a célula do <b>LADO DA PLATEIA</b> voltada pra cima
 *       (AprilTags 34-37, "Red Audience Tags") — Y menor (52).</li>
 *   <li><b>BLUE</b> começa com a célula do <b>LADO OPOSTO À PLATEIA</b> voltada pra cima
 *       (AprilTags 42-45, "Blue Scoring Tags") — Y maior (92).</li>
 * </ul>
 * Confirmado batendo o heading inicial calculado (90° RED / 270° BLUE) com o medido em campo.
 *
 * <h2>Estado dinâmico x staging fixo</h2>
 * {@link #getActiveCellPose} depende do estado mutável {@link DataStorage#activeCellIsAudienceSide}
 * (alternado pelo motorista a cada TIP) — é o certo pra mira durante a partida. Já
 * {@link #getInitialCellPose} é <b>puro</b>: sempre devolve o staging oficial acima, não importa
 * o que esteja em {@code DataStorage} no momento. Use o segundo pra qualquer cálculo que precise
 * valer ANTES da partida (ex.: pose inicial do autônomo) — assim ele não depende da ordem de
 * chamada de {@link #resetForMatchStart}, que só é garantida dentro de {@code Autos.prepareRoutine()}.
 *
 * @author LucasDiegoHD - Team #23069
 */
@Configurable
public final class HiveTargets {

    private HiveTargets() {
    }

    /** Célula RED do lado da plateia (AprilTags 34-37) — voltada pra cima no início da partida. */
    public static double RED_AUDIENCE_CELL_X = 60.0;
    public static double RED_AUDIENCE_CELL_Y = 52.0;

    /** Célula RED do lado oposto à plateia (AprilTags 30-33). */
    public static double RED_FAR_CELL_X = 60.0;
    public static double RED_FAR_CELL_Y = 92.0;

    /** Célula BLUE do lado da plateia (AprilTags 38-41). */
    public static double BLUE_AUDIENCE_CELL_X = 84.0;
    public static double BLUE_AUDIENCE_CELL_Y = 52.0;

    /** Célula BLUE do lado oposto à plateia (AprilTags 42-45) — voltada pra cima no início da partida. */
    public static double BLUE_FAR_CELL_X = 84.0;
    public static double BLUE_FAR_CELL_Y = 92.0;

    /**
     * Pose de mundo da célula ATUALMENTE voltada pra cima pra aliança informada, de acordo com
     * o estado alternado manualmente pelo motorista. Use durante a partida (mira em tempo real).
     */
    public static Pose getActiveCellPose(AllianceEnum alliance) {
        boolean audienceSideUp = DataStorage.activeCellIsAudienceSide;

        if (alliance == AllianceEnum.Red) {
            return audienceSideUp
                    ? new Pose(RED_AUDIENCE_CELL_X, RED_AUDIENCE_CELL_Y, 0)
                    : new Pose(RED_FAR_CELL_X, RED_FAR_CELL_Y, 0);
        }

        return audienceSideUp
                ? new Pose(BLUE_AUDIENCE_CELL_X, BLUE_AUDIENCE_CELL_Y, 0)
                : new Pose(BLUE_FAR_CELL_X, BLUE_FAR_CELL_Y, 0);
    }

    /** Overload usando a aliança ativa em {@link DataStorage#alliance}. */
    public static Pose getActiveCellPose() {
        return getActiveCellPose(DataStorage.alliance);
    }

    /**
     * Pose de mundo da célula que O MANUAL diz que JÁ começa voltada pra cima pra essa aliança
     * (Section 10.3.1). Ao contrário de {@link #getActiveCellPose}, NÃO lê {@code DataStorage}
     * — é pura, sempre dá o mesmo resultado pro mesmo {@code alliance}, não importa o que
     * aconteceu (ou não) antes na partida. Use isso pra qualquer coisa que precise ser correta
     * mesmo se {@link #resetForMatchStart} ainda não tiver rodado (ex.: pose inicial do TeleOp
     * quando ligado sem Autônomo antes).
     */
    public static Pose getInitialCellPose(AllianceEnum alliance) {
        return (alliance == AllianceEnum.Red)
                ? new Pose(RED_AUDIENCE_CELL_X, RED_AUDIENCE_CELL_Y, 0)
                : new Pose(BLUE_FAR_CELL_X, BLUE_FAR_CELL_Y, 0);
    }

    /**
     * Alterna manualmente qual lado do HIVE está pontuável. Chamado pelo botão de toggle no
     * TeleOp quando o motorista percebe (ou acabou de causar) um HIVE TIP.
     */
    public static void toggleActiveCell() {
        DataStorage.activeCellIsAudienceSide = !DataStorage.activeCellIsAudienceSide;
    }

    /**
     * Reseta o estado pro staging oficial de início de partida (ver tabela na doc da classe).
     * Chame isso em {@code Autos.prepareRoutine()} assim que a aliança for confirmada — NÃO
     * chame no início do TeleOp, ou você perde o estado acumulado durante o AUTO.
     */
    public static void resetForMatchStart(AllianceEnum alliance) {
        DataStorage.activeCellIsAudienceSide = (alliance == AllianceEnum.Red);
    }

    /**
     * Heading (radianos, convenção atan2 padrão) apontando de uma pose de origem até a célula
     * atualmente ativa (estado dinâmico — ver {@link #getActiveCellPose}). Use durante a partida.
     */
    public static double headingTowardsActiveCell(double fromX, double fromY, AllianceEnum alliance) {
        Pose target = getActiveCellPose(alliance);
        return Math.atan2(target.y() - fromY, target.x() - fromX);
    }

    /**
     * Heading (radianos) apontando de uma pose de origem até a célula que o manual diz que já
     * começa voltada pra cima (estado fixo — ver {@link #getInitialCellPose}). Use pra montar a
     * pose inicial do autônomo/TeleOp, já que precisa valer ANTES de qualquer TIP acontecer.
     */
    public static double headingTowardsInitialCell(double fromX, double fromY, AllianceEnum alliance) {
        Pose target = getInitialCellPose(alliance);
        return Math.atan2(target.y() - fromY, target.x() - fromX);
    }
}
