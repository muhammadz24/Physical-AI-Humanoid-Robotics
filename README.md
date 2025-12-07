# Physical AI & Humanoid Robotics - Interactive Textbook

An AI-native interactive textbook covering Physical AI, ROS 2, and Humanoid Robotics with integrated RAG chatbot for enhanced learning.

## 🚀 Features

- **6 Comprehensive Chapters**: From fundamentals to advanced capstone project
- **Interactive Code Examples**: Syntax highlighting, line numbers, and copy buttons
- **Dark Mode Support**: Read comfortably in any lighting condition
- **RAG Chatbot**: Ask questions and get answers grounded in textbook content (coming soon)
- **Self-Assessment**: Questions at the end of each chapter
- **Mobile Responsive**: Learn on any device

## 📚 Chapters

1. Introduction to Physical AI
2. Basics of Humanoid Robotics
3. ROS 2 Fundamentals
4. Digital Twin Simulation (Gazebo + Isaac Sim)
5. Vision-Language-Action Systems
6. Capstone: Simple AI-Robot Pipeline

## 🛠️ Tech Stack

- **Frontend**: Docusaurus v3.x (React-based static site generator)
- **Backend**: FastAPI (Python 3.10+)
- **Vector Database**: Qdrant Cloud (free tier)
- **Database**: Neon Postgres (free tier)
- **Embeddings**: sentence-transformers/all-MiniLM-L6-v2 (local)
- **Deployment**: GitHub Pages (frontend), Railway (backend)

## 🏃 Quick Start

### Prerequisites
- Node.js 18.x or 20.x
- Python 3.10+
- npm or yarn

### Installation

```bash
# Install frontend dependencies
npm install

# Install backend dependencies (optional, for RAG chatbot)
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### Development

```bash
# Start Docusaurus development server
npm start

# Build for production
npm run build

# Serve production build locally
npm run serve
```

### Backend Setup (Optional)

```bash
cd backend
cp .env.example .env
# Edit .env with your Qdrant and Neon credentials

# Run FastAPI server
uvicorn src.main:app --reload
```

## 📖 Documentation

- **Specification**: See `specs/textbook-generation/spec.md`
- **Implementation Plan**: See `specs/textbook-generation/plan.md`
- **Tasks**: See `specs/textbook-generation/tasks.md`

## 🎯 Project Goals

This project demonstrates:
- **Free-tier architecture**: Entire system runs on free services
- **Spec-driven development**: Complete specification before implementation
- **AI-native design**: RAG chatbot integrated from the start
- **Educational focus**: Content quality over feature bloat

## 📜 License

- **Content**: CC BY-SA 4.0
- **Code**: MIT License

## 🤝 Contributing

This is an educational project. Contributions are welcome! Please ensure:
- All code examples are tested on Ubuntu 22.04 + ROS 2 Humble
- Content follows the constitution guidelines
- Technical accuracy verified against official documentation

## 🔗 Resources

- [ROS 2 Documentation](https://docs.ros.org)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim)
- [Docusaurus Documentation](https://docusaurus.io/docs)

---

**Built with ❤️ for Physical AI learners everywhere**
