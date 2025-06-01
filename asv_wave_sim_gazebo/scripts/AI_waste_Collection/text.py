#!/usr/bin/env python3

import rospy
import math
import time
from asv_wave_sim_gazebo.msg import WasteDetection
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest

class ProfessionalWasteCollector:
    def __init__(self):
        rospy.init_node('professional_waste_collector', anonymous=True)

        # Wait for Gazebo services
        try:
            rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=10)
            rospy.wait_for_service('/gazebo/clear_body_wrenches', timeout=10)
        except rospy.ROSException as e:
            rospy.logerr(f"Gazebo services not available: {e}")
            return

        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.clear_wrench = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)

        # PARAMÈTRES PID OPTIMISÉS POUR MICRO-AJUSTEMENTS PROGRESSIFS
        self.kp_angular = 0.3    # Gain proportionnel RÉDUIT pour micro-ajustements
        self.ki_angular = 0.05   # Gain intégral faible pour stabilité
        self.kd_angular = 0.1    # Gain dérivé réduit pour éviter oscillations

        self.kp_linear = 0.4     # Gain proportionnel modéré pour avancement contrôlé
        self.ki_linear = 0.03    # Gain intégral faible
        self.kd_linear = 0.1     # Gain dérivé réduit

        # Limites de commande
        self.max_torque = 0.20
        self.min_torque = -0.20
        self.base_speed = -0.30

        # Variables d'état du régulateur
        self.angular_error_integral = 0.0
        self.linear_error_integral = 0.0
        self.previous_angular_error = 0.0
        self.previous_linear_error = 0.0
        self.previous_time = time.time()

        # Identification des ventilateurs
        self.fan_right = "boatcleaningc::fandroit"
        self.fan_left = "boatcleaningc::fangauche"

        # État du système
        self.collection_count = 0
        self.regulation_mode = "SEARCH"  # SEARCH, ORIENT, APPROACH, COLLECT
        self.target_position = {"x": 0, "y": 0}  # Position cible du déchet
        self.regulation_active = False

        # Variables manquantes pour compatibilité
        self.strong_turn = -0.15
        self.medium_turn = -0.10
        self.forward_speed = -0.08

        # Subscribe to waste detection
        rospy.Subscriber('/waste_detection', WasteDetection, self.waste_detection_callback)
        rospy.loginfo("Professional Waste Collector with PID Regulation started!")

    def apply_torque(self, link_name, torque):
        try:
            self.clear_wrench(link_name)
            if abs(torque) > 0.001:
                req = ApplyBodyWrenchRequest()
                req.body_name = link_name
                req.reference_frame = "world"
                req.wrench.torque.x = torque
                req.duration = rospy.Duration(-1)
                self.apply_wrench(req)
                rospy.loginfo(f"Applied torque {torque} on {link_name}")
            else:
                rospy.loginfo(f"Cleared torque on {link_name}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to apply torque on {link_name}: {e}")

    def stop_fans(self):
        self.apply_torque(self.fan_right, 0.0)
        self.apply_torque(self.fan_left, 0.0)
        rospy.loginfo("Boat stopped.")

    def move_forward_slowly(self):
        """Move forward slowly for search and approach"""
        self.apply_torque(self.fan_right, self.base_speed)
        self.apply_torque(self.fan_left, self.base_speed)
        rospy.loginfo("Boat moving forward slowly.")

    def calculate_waste_position_error(self, left_count, right_count, upper_middle_count, bottom_middle_count):
        """
        🎯 MAINTENIR LE DÉCHET AU CENTRE DE BOTTOM MIDDLE POUR LE FILET
        Objectif: Garder le déchet centré dans Bottom Middle pour une collecte optimale
        """

        # 🎯 PRIORITÉ 1: DÉCHET EN BOTTOM MIDDLE - MAINTENIR AU CENTRE ET AVANCER
        if bottom_middle_count > 0:
            # Le déchet est en Bottom Middle, maintenir au centre ET avancer pour le ramasser
            angular_error = 0.0
            linear_error = 0.8  # AVANCEMENT FORT pour ramasser le déchet

            # Si le déchet est aussi sur les côtés, corriger pour le centrer
            if left_count > 0 and right_count == 0:
                angular_error = 0.2  # Correction douce vers la gauche pour centrer
                rospy.loginfo("🎯 RAMASSAGE: Déchet en Bottom Middle + gauche - centrage + avancement")
            elif right_count > 0 and left_count == 0:
                angular_error = -0.2  # Correction douce vers la droite pour centrer
                rospy.loginfo("🎯 RAMASSAGE: Déchet en Bottom Middle + droite - centrage + avancement")
            else:
                rospy.loginfo("🎯 RAMASSAGE: Déchet centré en Bottom Middle - AVANCEMENT POUR COLLECTE!")

            return angular_error, linear_error

        # 🎯 PRIORITÉ 2: MICRO-AJUSTEMENTS PROGRESSIFS POUR CENTRER LE DÉCHET
        angular_error = 0.0
        linear_error = 0.0

        # Détection du déchet et calcul des MICRO-AJUSTEMENTS
        waste_detected = (left_count > 0 or right_count > 0 or upper_middle_count > 0)

        if waste_detected:
            rospy.loginfo("🎯 DÉCHET DÉTECTÉ - MICRO-AJUSTEMENTS PROGRESSIFS!")

            # STRATÉGIE: MICRO-AJUSTEMENTS pour ne pas perdre le déchet de vue
            if left_count > 0 and right_count == 0:
                # Déchet à GAUCHE - MICRO-ROTATION à gauche
                if upper_middle_count > 0:
                    angular_error = 0.15   # MICRO-ajustement - déchet proche du centre
                    rospy.loginfo("🔄 MICRO-AJUSTEMENT: Déchet gauche + centre - micro-rotation gauche")
                else:
                    angular_error = 0.25   # Ajustement modéré - déchet plus loin
                    rospy.loginfo("🔄 MICRO-AJUSTEMENT: Déchet à gauche - rotation modérée gauche")

            elif right_count > 0 and left_count == 0:
                # Déchet à DROITE - MICRO-ROTATION à droite
                if upper_middle_count > 0:
                    angular_error = -0.15  # MICRO-ajustement - déchet proche du centre
                    rospy.loginfo("🔄 MICRO-AJUSTEMENT: Déchet droite + centre - micro-rotation droite")
                else:
                    angular_error = -0.25  # Ajustement modéré - déchet plus loin
                    rospy.loginfo("🔄 MICRO-AJUSTEMENT: Déchet à droite - rotation modérée droite")

            elif left_count > 0 and right_count > 0:
                # Déchet des DEUX CÔTÉS - Maintenir le cap et avancer
                angular_error = 0.0
                rospy.loginfo("🔄 MAINTIEN CAP: Déchet des deux côtés - maintien cap central")

            elif upper_middle_count > 0:
                # Déchet CENTRÉ en Upper Middle - Maintenir cap et avancer
                angular_error = 0.0
                rospy.loginfo("🔄 MAINTIEN CAP: Déchet centré en Upper Middle - maintien cap")

            # CALCUL DE L'AVANCEMENT PROGRESSIF
            if upper_middle_count > 0:
                # Déchet en Upper Middle - avancement modéré vers Bottom Middle
                linear_error = 0.4
                rospy.loginfo("⬇️ AVANCEMENT: Déchet en Upper Middle - poussée modérée vers Bottom Middle")
            elif left_count > 0 or right_count > 0:
                # Déchet sur les côtés - avancement lent pour maintenir la détection
                linear_error = 0.3
                rospy.loginfo("⬆️ APPROCHE LENTE: Déchet sur côtés - approche lente pour maintenir détection")

        else:
            # AUCUN DÉCHET DÉTECTÉ - Mode recherche
            angular_error = 0.0
            linear_error = 0.5  # Avancement modéré pour recherche
            rospy.loginfo("🔍 RECHERCHE: Aucun déchet - avancement de recherche modéré")

        rospy.loginfo(f"🎯 MICRO-COMMANDES: Angular={angular_error:.2f}, Linear={linear_error:.2f}")
        return angular_error, linear_error

    def pid_controller(self, angular_error, linear_error):
        """
        RÉGULATEUR PID PROFESSIONNEL
        Calcule les commandes moteur basées sur les erreurs
        """
        current_time = time.time()
        dt = current_time - self.previous_time

        if dt <= 0:
            dt = 0.01  # Éviter division par zéro

        # === RÉGULATION ANGULAIRE (ORIENTATION) ===
        # Terme proportionnel
        angular_p = self.kp_angular * angular_error

        # Terme intégral (accumulation de l'erreur)
        self.angular_error_integral += angular_error * dt
        angular_i = self.ki_angular * self.angular_error_integral

        # Terme dérivé (vitesse de changement de l'erreur)
        angular_derivative = (angular_error - self.previous_angular_error) / dt
        angular_d = self.kd_angular * angular_derivative

        # Commande angulaire totale
        angular_command = angular_p + angular_i + angular_d

        # === RÉGULATION LINÉAIRE (AVANCEMENT) ===
        # Terme proportionnel
        linear_p = self.kp_linear * linear_error

        # Terme intégral
        self.linear_error_integral += linear_error * dt
        linear_i = self.ki_linear * self.linear_error_integral

        # Terme dérivé
        linear_derivative = (linear_error - self.previous_linear_error) / dt
        linear_d = self.kd_linear * linear_derivative

        # Commande linéaire totale
        linear_command = linear_p + linear_i + linear_d

        # Saturation des commandes
        angular_command = max(min(angular_command, self.max_torque), self.min_torque)
        linear_command = max(min(linear_command, self.max_torque), self.min_torque)

        # Mise à jour des variables d'état
        self.previous_angular_error = angular_error
        self.previous_linear_error = linear_error
        self.previous_time = current_time

        return angular_command, linear_command

    def apply_differential_control(self, angular_command, linear_command):
        """
        COMMANDE DIFFÉRENTIELLE PROFESSIONNELLE
        Convertit les commandes angulaire/linéaire en commandes moteur
        """
        # Calcul des vitesses moteur différentielles
        right_motor = self.base_speed + linear_command - angular_command
        left_motor = self.base_speed + linear_command + angular_command

        # Saturation des moteurs
        right_motor = max(min(right_motor, self.max_torque), self.min_torque)
        left_motor = max(min(left_motor, self.max_torque), self.min_torque)

        rospy.loginfo(f"PID Control: Angular={angular_command:.3f}, Linear={linear_command:.3f}")
        rospy.loginfo(f"Motors: Right={right_motor:.3f}, Left={left_motor:.3f}")

        return right_motor, left_motor

    def regulate_towards_waste(self, left_count, right_count, upper_middle_count, bottom_middle_count):
        """
        RÉGULATION PROFESSIONNELLE POUR MAINTENIR LE DÉCHET EN BOTTOM MIDDLE
        Objectif: Orienter le bateau pour que le déchet reste TOUJOURS dans Bottom Middle
        """

        # OBJECTIF PRINCIPAL: Déchet en Bottom Middle = CONTINUER L'AVANCEMENT POUR RAMASSER
        if bottom_middle_count >= 1:
            rospy.loginfo("🎯 RAMASSAGE: Déchet en Bottom Middle - AVANCEMENT POUR COLLECTE!")

            # Ne pas s'arrêter immédiatement, continuer d'avancer pour ramasser
            # Calculer l'erreur pour maintenir le centrage et avancer
            angular_error, linear_error = self.calculate_waste_position_error(
                left_count, right_count, upper_middle_count, bottom_middle_count
            )

            # Appliquer la régulation PID pour maintenir le centrage et avancer
            angular_command, linear_command = self.pid_controller(angular_error, linear_error)
            right_motor, left_motor = self.apply_differential_control(angular_command, linear_command)

            # Appliquer les commandes aux moteurs
            self.apply_torque(self.fan_right, right_motor)
            self.apply_torque(self.fan_left, left_motor)

            rospy.loginfo(f"🎯 RAMASSAGE ACTIF: Angular={angular_error:.2f}, Linear={linear_error:.2f}")
            rospy.loginfo(f"⚙️ MOTEURS RAMASSAGE: Droite={right_motor:.3f}, Gauche={left_motor:.3f}")

            self.regulation_active = True

            # Compter la collecte après un certain temps d'avancement
            # (vous pouvez ajuster cette logique selon vos besoins)
            return False

        # STRATÉGIE: Calculer l'erreur pour amener le déchet vers Bottom Middle
        angular_error, linear_error = self.calculate_waste_position_error(
            left_count, right_count, upper_middle_count, bottom_middle_count
        )

        # RÉGULATION PID PROFESSIONNELLE
        angular_command, linear_command = self.pid_controller(angular_error, linear_error)

        # APPLICATION DE LA COMMANDE DIFFÉRENTIELLE
        right_motor, left_motor = self.apply_differential_control(angular_command, linear_command)

        # APPLIQUER LES COMMANDES AUX MOTEURS
        self.apply_torque(self.fan_right, right_motor)
        self.apply_torque(self.fan_left, left_motor)

        # AFFICHAGE DU STATUT D'ORIENTATION VERS LE DÉCHET
        rospy.loginfo(f" ORIENTATION VERS DÉCHET: Angular={angular_error:.2f}, Linear={linear_error:.2f}")
        rospy.loginfo(f" COMMANDES MOTEUR: Droite={right_motor:.3f}, Gauche={left_motor:.3f}")

        # DÉTERMINER LE TYPE D'ORIENTATION EN COURS
        waste_detected = (left_count > 0 or right_count > 0 or upper_middle_count > 0)

        if waste_detected:
            if left_count >= 1 and right_count == 0:
                if upper_middle_count >= 1:
                    rospy.loginfo("ORIENTATION: Déchet à gauche + centre - rotation modérée VERS LA GAUCHE")
                else:
                    rospy.loginfo(" ORIENTATION: Déchet à gauche - rotation forte VERS LA GAUCHE")
            elif right_count >= 1 and left_count == 0:
                if upper_middle_count >= 1:
                    rospy.loginfo("ORIENTATION: Déchet à droite + centre - rotation modérée VERS LA DROITE")
                else:
                    rospy.loginfo("ORIENTATION: Déchet à droite - rotation forte VERS LA DROITE")
            elif upper_middle_count >= 1:
                rospy.loginfo("ORIENTATION: Déchet centré - guidage direct vers Bottom Middle")
            elif left_count >= 1 and right_count >= 1:
                rospy.loginfo(" ORIENTATION: Déchet des deux côtés - maintien du cap central")
        else:
            rospy.loginfo(" RECHERCHE: Aucun déchet détecté - avancement de recherche")

        self.regulation_active = True
        return False

    def waste_detection_callback(self, msg):
        # Initialize counts for each section
        upper_middle_count = 0
        bottom_middle_count = 0
        left_count = 0
        right_count = 0
        for section in msg.sections:
            if section.section == "Upper Middle":
                upper_middle_count = section.count
            elif section.section == "Bottom Middle":
                bottom_middle_count = section.count
            elif section.section == "Left":
                left_count = section.count
            elif section.section == "Right":
                right_count = section.count

        rospy.loginfo(f"Waste detection: Left={left_count}, Right={right_count}, Upper Middle={upper_middle_count}, Bottom Middle={bottom_middle_count}")

        # SYSTÈME DE RÉGULATION INTELLIGENT
        collected = self.regulate_towards_waste(left_count, right_count, upper_middle_count, bottom_middle_count)

        if collected:
            rospy.loginfo(f"Total waste collected: {self.collection_count}")
            # Pause courte après collecte avant de reprendre la recherche
            rospy.sleep(1.0)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ProfessionalWasteCollector()
        node.run()
    except rospy.ROSInterruptException:
        pass