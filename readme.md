# 🚀 ViNT + Habitat-ROS Integration

This package 🌉 bridges the **ViNT (Visual Navigation Transformer)** continuous navigation model with **Habitat’s** discrete simulation via ROS middleware, replacing mock-testing nodes with a real, learned visual navigation stack.

---

## 📦 Repositories

- **Visual Navigation Transformer**  
  🔗 https://github.com/robodhruv/visualnav-transformer

- **Habitat-ROS**  
  🔗 https://github.com/ericchen321/ros_x_habitat

- **This Integration**  
  🔗 https://github.com/briankwak810/vint_x_habitat

---

## 📋 Prerequisites

- **OS:** Ubuntu 20.04  
- **ROS:**  
  - Noetic (for Habitat-ROS)  
  - Humble (or later) (for ROS 2/ViNT)  
- **Habitat-Sim & Habitat-Lab:** Installed per official docs  
- **Docker:** (optional, for the ViNT container)  

---

## v0: Naive General Navigation Models

- **Pretrained** GNMs on previous research, e.g. ViNT[https://arxiv.org/abs/2306.14846], NoMaD[https://arxiv.org/abs/2310.07896] can be deployed and teleoperated **fully online**.
- MP3D scene datasets can be explored(zero-shot) and navigated(with prior topological map at hand).
- **Below:** Goal-directed navigation done by NoMaD model with goal image given as shown.
  <table>
  <tr>
    <td>
      <!-- left cell: video -->
      <video controls width="400">
        <source src="./files/Naive NoMaD with zero fine-tuning.mp4" type="video/mp4">
        Your browser doesn’t support HTML5 video. 
        <a href="./files/Naive NoMaD with zero fine-tuning.mp4">Download the video</a> instead.
      </video>
    </td>
    <td>
      <!-- right cell: image -->
      <img src="./files/Goal for Naive NoMaD with zero fine-tuning.png" alt="Diagram" width="400" />
    </td>
  </tr>
</table>

- **Issues and future development:** Solve physical crashing issues, fine-tuning checkpoints for navigation in unseen environment